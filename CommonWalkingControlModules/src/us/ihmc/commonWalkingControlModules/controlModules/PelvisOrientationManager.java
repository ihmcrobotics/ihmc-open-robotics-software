package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisOrientationTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class PelvisOrientationManager
{
   private static final double defaultTrajectoryTime = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameQuaternion desiredPelvisOrientation = new YoFrameQuaternion("desiredPelvis", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularVelocity = new YoFrameVector("desiredPelvisAngularVelocity", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", worldFrame, registry);

   private final DoubleYoVariable swingPelvisYaw = new DoubleYoVariable("swingPelvisYaw", registry);
   private final DoubleYoVariable swingPelvisYawScale = new DoubleYoVariable("swingPelvisYawScale", registry);

   private final DoubleYoVariable initialPelvisOrientationTime = new DoubleYoVariable("initialPelvisOrientationTime", registry);
   private final YoFrameQuaternion initialPelvisOrientation = new YoFrameQuaternion("initialPelvis", worldFrame, registry);
   private final YoFrameQuaternion finalPelvisOrientation = new YoFrameQuaternion("finalPelvis", worldFrame, registry);
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final DoubleYoVariable initialPelvisOrientationOffsetTime = new DoubleYoVariable("initialPelvisOrientationOffsetTime", registry);
   private final YoFrameQuaternion desiredPelvisOrientationOffset;

   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationOffsetTrajectoryGenerator;

   private final YoFrameQuaternion desiredPelvisOrientationWithOffset = new YoFrameQuaternion("desiredPelvisOrientationWithOffset", worldFrame, registry);

   private final DoubleYoVariable yoTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;
   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isPelvisOrientationOffsetTrajectoryStopped", registry);

   public PelvisOrientationManager(WalkingControllerParameters walkingControllerParameters, MomentumBasedController momentumBasedController,
         PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber,
         StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber, YoVariableRegistry parentRegistry)
   {
      yoTime = momentumBasedController.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      pelvisFrame = referenceFrames.getPelvisFrame();
      this.pelvisOrientationTrajectoryMessageSubscriber = pelvisOrientationTrajectoryMessageSubscriber;
      this.stopAllTrajectoryMessageSubscriber = stopAllTrajectoryMessageSubscriber;

      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);
      double defaultStepTime = walkingControllerParameters.getDefaultSwingTime();
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(defaultStepTime);

      pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(ankleZUpFrames.get(robotSide));

      YoOrientationPIDGainsInterface pelvisOrientationControlGains = walkingControllerParameters.createPelvisOrientationControlGains(registry);
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();
      orientationFeedbackControlCommand.set(elevator, pelvis);
      orientationFeedbackControlCommand.setGains(pelvisOrientationControlGains);

      desiredPelvisFrame = new ReferenceFrame("desiredPelvisFrame", worldFrame)
      {
         private static final long serialVersionUID = -1472151257649344278L;

         private final Quat4d rotationToParent = new Quat4d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            pelvisFrame.getTransformToDesiredFrame(transformToParent, parentFrame);
            desiredPelvisOrientation.get(rotationToParent);
            transformToParent.setRotation(rotationToParent);
         }
      };

      desiredPelvisOrientationOffset = new YoFrameQuaternion("desiredPelvis", "Offset", desiredPelvisFrame, registry);

      boolean allowMultipleFrames = true;
      waypointOrientationOffsetTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("pelvisOffset", 15, allowMultipleFrames,
            desiredPelvisFrame, registry);
      waypointOrientationOffsetTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
   }

   private void initialize(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientationTime.set(yoTime.getDoubleValue());

      pelvisOrientationTrajectoryGenerator.switchTrajectoryFrame(desiredTrajectoryFrame);

      initialPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setInitialOrientation(tempOrientation);

      finalPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setFinalOrientation(tempOrientation);

      pelvisOrientationTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
   }

   public void compute()
   {
      double deltaTime = yoTime.getDoubleValue() - initialPelvisOrientationTime.getDoubleValue();
      pelvisOrientationTrajectoryGenerator.compute(deltaTime);
      pelvisOrientationTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
      desiredPelvisFrame.update();

      handleStopAllTrajectoryMessage();
      handlePelvisOrientationTrajectoryMessages();

      if (isTrajectoryStopped.getBooleanValue())
      {
         waypointOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
         tempAngularVelocity.setToZero();
         tempAngularAcceleration.setToZero();
      }
      else
      {
         double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
         waypointOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
         waypointOrientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
      }

      desiredPelvisOrientationOffset.set(tempOrientation);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientationWithOffset.set(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      desiredPelvisOrientationWithOffset.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      desiredPelvisAngularAcceleration.getFrameTupleIncludingFrame(tempAngularAcceleration);
      
      orientationFeedbackControlCommand.set(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
   }

   public void goToHomeFromCurrentDesired()
   {
      goToHomeFromCurrentDesired(defaultTrajectoryTime);
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      waypointOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
      tempOrientation.changeFrame(desiredPelvisFrame);
      tempAngularVelocity.setToZero(desiredPelvisFrame);

      waypointOrientationOffsetTrajectoryGenerator.clear();
      waypointOrientationOffsetTrajectoryGenerator.switchTrajectoryFrame(desiredPelvisFrame);
      waypointOrientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      tempOrientation.setToZero(desiredPelvisFrame);
      waypointOrientationOffsetTrajectoryGenerator.appendWaypoint(trajectoryTime, tempOrientation, tempAngularVelocity);
      waypointOrientationOffsetTrajectoryGenerator.initialize();

      isTrajectoryStopped.set(false);
   }

   private void handleStopAllTrajectoryMessage()
   {
      if (stopAllTrajectoryMessageSubscriber == null || !stopAllTrajectoryMessageSubscriber.pollMessage(this))
         return;
      isTrajectoryStopped.set(true);
   }

   public void handlePelvisTrajectoryMessage(PelvisTrajectoryMessage message)
   {
      handleSO3Waypoints(message.getWaypoints());
   }

   private void handlePelvisOrientationTrajectoryMessages()
   {
      if (pelvisOrientationTrajectoryMessageSubscriber != null && pelvisOrientationTrajectoryMessageSubscriber.isNewTrajectoryMessageAvailable())
         handleSO3Waypoints(pelvisOrientationTrajectoryMessageSubscriber.pollMessage().getWaypoints());
   }

   public void handleSO3Waypoints(SO3WaypointInterface[] waypoints)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      if (waypoints[0].getTime() > 1.0e-5)
      {
         waypointOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         waypointOrientationOffsetTrajectoryGenerator.clear();
         waypointOrientationOffsetTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         waypointOrientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }
      else
      {
         waypointOrientationOffsetTrajectoryGenerator.clear();
         waypointOrientationOffsetTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      }

      waypointOrientationOffsetTrajectoryGenerator.appendWaypoints(waypoints);
      waypointOrientationOffsetTrajectoryGenerator.changeFrame(desiredPelvisFrame);
      waypointOrientationOffsetTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(desiredPelvisFrame);
      tempAngularVelocity.setToZero(desiredPelvisFrame);
      waypointOrientationOffsetTrajectoryGenerator.clear();
      waypointOrientationOffsetTrajectoryGenerator.switchTrajectoryFrame(desiredPelvisFrame);
      waypointOrientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      waypointOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setToHoldCurrentInWorldFrame()
   {
      tempOrientation.setToZero(pelvisFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize(worldFrame);
   }

   public void prepareForLocomotion()
   {
      desiredPelvisOrientationWithOffset.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(initialPelvisOrientation.getReferenceFrame());
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      resetOrientationOffset();
      initialize(worldFrame);
   }

   public void setToHoldCurrentDesiredInWorldFrame()
   {
      setToHoldCurrentDesired(worldFrame);
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      setToHoldCurrentDesired(midFeetZUpFrame);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      setToHoldCurrentDesired(ankleZUpFrames.get(supportSide));
   }

   public void setToHoldCurrentDesired(ReferenceFrame desiredTrajectoryFrame)
   {
      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);

      initialize(desiredTrajectoryFrame);
   }

   /** Go instantly to zero, no smooth interpolation. */
   public void setToZeroInSupportFoot(RobotSide supportSide)
   {
      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(supportSide);
      tempOrientation.setToZero(supportAnkleZUp);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize(supportAnkleZUp);
   }

   /** Go instantly to zero, no smooth interpolation. */
   public void setToZeroInMidFeetZUpFrame()
   {
      tempOrientation.setToZero(midFeetZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize(midFeetZUpFrame);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      ReferenceFrame otherAnkleZUpFrame = ankleZUpFrames.get(supportSide.getOppositeSide());
      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportSide);

      tempOrientation.setToZero(otherAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawOtherFoot = tempOrientation.getYaw();

      tempOrientation.setToZero(supportAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawOtherFoot, yawSupportFoot);

      finalPelvisOrientation.set(finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(supportAnkleZUpFrame);
   }

   /** Move towards zero smoothly within the given swing time */
   public void moveToZeroInSupportFoot(RobotSide supportSide)
   {
      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(supportSide);
      tempOrientation.setToZero(supportAnkleZUp);
      tempOrientation.changeFrame(worldFrame);
      finalPelvisOrientation.set(tempOrientation);

      initialize(supportAnkleZUp);
   }

   private final FramePoint upcomingFootstepLocation = new FramePoint();
   private final FrameOrientation upcomingFootstepOrientation = new FrameOrientation();

   public void setWithUpcomingFootstep(Footstep upcomingFootstep)
   {
      RobotSide upcomingFootstepSide = upcomingFootstep.getRobotSide();

      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      upcomingFootstep.getOrientationIncludingFrame(upcomingFootstepOrientation);
      upcomingFootstepOrientation.changeFrame(worldFrame);
      tempOrientation.setToZero(ankleZUpFrames.get(upcomingFootstepSide.getOppositeSide()));
      tempOrientation.changeFrame(worldFrame);

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(upcomingFootstepOrientation.getYaw(), tempOrientation.getYaw());

      upcomingFootstep.getPositionIncludingFrame(upcomingFootstepLocation);
      upcomingFootstepLocation.changeFrame(ankleZUpFrames.get(upcomingFootstepSide.getOppositeSide()));

      double desiredSwingPelvisYawAngle = 0.0;
      if (Math.abs(upcomingFootstepLocation.getX()) > 0.1)
      {
         desiredSwingPelvisYawAngle = Math.atan2(upcomingFootstepLocation.getY(), upcomingFootstepLocation.getX());
         desiredSwingPelvisYawAngle -= upcomingFootstepSide.negateIfRightSide(Math.PI / 2.0);
      }
      swingPelvisYaw.set(desiredSwingPelvisYawAngle);

      finalPelvisOrientation.set(finalDesiredPelvisYawAngle + swingPelvisYawScale.getDoubleValue() * desiredSwingPelvisYawAngle, 0.0, 0.0);

      initialize(worldFrame);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}

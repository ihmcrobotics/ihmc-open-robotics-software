package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAngularAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
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
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.robotics.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
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

   private final DoubleYoVariable offsetTrajectoryTime = new DoubleYoVariable("offsetTrajectoryTime", registry);
   private final YoFrameQuaternion initialPelvisOrientationOffset;
   private final YoFrameQuaternion finalPelvisOrientationOffset;

   private final BooleanYoVariable isUsingWaypointTrajectory;
   private OrientationTrajectoryGenerator activeOrientationOffsetTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator pelvisWaypointsOrientationOffsetTrajectoryGenerator;
   private final OrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;

   private final YoFrameQuaternion desiredPelvisOrientationWithOffset = new YoFrameQuaternion("desiredPelvisOrientationWithOffset", worldFrame, registry);

   private final DoubleYoVariable yoTime;

   private final OrientationTrajectoryData orientationTrajectoryData = new OrientationTrajectoryData();
   private final RootJointAngularAccelerationControlModule rootJointAngularAccelerationControlModule;

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final PelvisPoseProvider pelvisPoseProvider;

   public PelvisOrientationManager(WalkingControllerParameters walkingControllerParameters, MomentumBasedController momentumBasedController,
         PelvisPoseProvider desiredPelvisPoseProvider, YoVariableRegistry parentRegistry)
   {
      yoTime = momentumBasedController.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      pelvisFrame = referenceFrames.getPelvisFrame();
      this.pelvisPoseProvider = desiredPelvisPoseProvider;

      YoOrientationPIDGains pelvisOrientationControlGains = walkingControllerParameters.createPelvisOrientationControlGains(registry);
      rootJointAngularAccelerationControlModule = new RootJointAngularAccelerationControlModule(momentumBasedController, pelvisOrientationControlGains,
            registry);
      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);
      double defaultStepTime = walkingControllerParameters.getDefaultSwingTime();
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(defaultStepTime);

      pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(ankleZUpFrames.get(robotSide));

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

      offsetTrajectoryTime.set(defaultTrajectoryTime);
      initialPelvisOrientationOffset = new YoFrameQuaternion("initialPelvis", "Offset", desiredPelvisFrame, registry);
      finalPelvisOrientationOffset = new YoFrameQuaternion("finalPelvis", "Offset", desiredPelvisFrame, registry);
      OrientationProvider initialPelvisOrientationOffsetProvider = new YoQuaternionProvider(initialPelvisOrientationOffset);
      OrientationProvider finalPelvisOrientationOffsetProvider = new YoQuaternionProvider(finalPelvisOrientationOffset);
      DoubleProvider offsetTrajectoryTimeProvider = new YoVariableDoubleProvider(offsetTrajectoryTime);

      isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
      isUsingWaypointTrajectory.set(false);

      pelvisOrientationOffsetTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvisOffset", desiredPelvisFrame,
            offsetTrajectoryTimeProvider, initialPelvisOrientationOffsetProvider, finalPelvisOrientationOffsetProvider, registry);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
      activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;

      boolean allowMultipleFrames = false;
      pelvisWaypointsOrientationOffsetTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("pelvisWaypointsOffset", 15,
            allowMultipleFrames, desiredPelvisFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
   }

   private void initialize(ReferenceFrame desiredTrajectoryFrame)
   {
      isUsingWaypointTrajectory.set(false);
      activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;

      initialPelvisOrientationTime.set(yoTime.getDoubleValue());

      pelvisOrientationTrajectoryGenerator.switchTrajectoryFrame(desiredTrajectoryFrame);

      initialPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setInitialOrientation(tempOrientation);

      finalPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setFinalOrientation(tempOrientation);

      pelvisOrientationTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
   }

   public void clearProvider()
   {
      pelvisPoseProvider.clearOrientation();
   }

   public void compute()
   {
      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeOrientationOffsetTrajectoryGenerator = pelvisWaypointsOrientationOffsetTrajectoryGenerator;
         else
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
      }

      updateDesireds();

      rootJointAngularAccelerationControlModule.doControl(orientationTrajectoryData);
   }

   private void updateDesireds()
   {
      double deltaTime = yoTime.getDoubleValue() - initialPelvisOrientationTime.getDoubleValue();
      pelvisOrientationTrajectoryGenerator.compute(deltaTime);
      pelvisOrientationTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
      desiredPelvisFrame.update();

      if (pelvisPoseProvider != null)
      {
         if (pelvisPoseProvider.checkForHomeOrientation())
         {
            initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
            offsetTrajectoryTime.set(pelvisPoseProvider.getTrajectoryTime());
            activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
            initialPelvisOrientationOffset.set(tempOrientation);
            finalPelvisOrientationOffset.set(0.0, 0.0, 0.0);
            pelvisOrientationOffsetTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
         }
         else if (pelvisPoseProvider.checkForNewOrientation())
         {
            initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
            offsetTrajectoryTime.set(pelvisPoseProvider.getTrajectoryTime());
            activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
            initialPelvisOrientationOffset.set(tempOrientation);
            FrameOrientation pelvisOrientationProvided = pelvisPoseProvider.getDesiredPelvisOrientation(desiredPelvisFrame);

            pelvisOrientationProvided.changeFrame(desiredPelvisFrame);
            tempOrientation.setIncludingFrame(pelvisOrientationProvided);
            finalPelvisOrientationOffset.set(tempOrientation);

            pelvisOrientationOffsetTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
         }
         else if (pelvisPoseProvider.checkForNewOrientationWithWaypoints())
         {
            initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

            WaypointOrientationTrajectoryData trajectoryData = pelvisPoseProvider.getDesiredPelvisOrientationWithWaypoints();

            int lastIndex = trajectoryData.getTimeAtWaypoints().length - 1;
            // it is not really the last time, since we have the "settling"
            double totalTime = trajectoryData.getTimeAtWaypoints()[lastIndex];

            offsetTrajectoryTime.set(totalTime);
            activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
            initialPelvisOrientationOffset.set(tempOrientation);

            FrameOrientation pelvisOrientationProvided = new FrameOrientation(ReferenceFrame.getWorldFrame(), trajectoryData.getOrientations()[lastIndex]);

            pelvisOrientationProvided.changeFrame(desiredPelvisFrame);
            tempOrientation.setIncludingFrame(pelvisOrientationProvided);
            finalPelvisOrientationOffset.set(tempOrientation);

            pelvisOrientationOffsetTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;

            //TODO @DAVIDE
            /*
             * activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
             * pelvisWaypointsOrientationOffsetTrajectoryGenerator.clear();
             * pelvisWaypointsOrientationOffsetTrajectoryGenerator.
             * appendWaypoint(0.0, tempOrientation);
             * WaypointOrientationTrajectoryData trajectoryData =
             * pelvisPoseProvider.getDesiredPelvisOrientationWithWaypoints();
             * trajectoryData.changeFrame(desiredPelvisFrame);
             * pelvisWaypointsOrientationOffsetTrajectoryGenerator.
             * appendWaypoints(trajectoryData);
             * pelvisWaypointsOrientationOffsetTrajectoryGenerator.initialize();
             * isUsingWaypointTrajectory.set(true);
             * activeOrientationOffsetTrajectoryGenerator =
             * pelvisWaypointsOrientationOffsetTrajectoryGenerator;
             */
         }
      }

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      activeOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      activeOrientationOffsetTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

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
      orientationTrajectoryData.set(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(desiredPelvisFrame);
      initialPelvisOrientationOffset.set(tempOrientation);
      finalPelvisOrientationOffset.set(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
      isUsingWaypointTrajectory.set(false);
      activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
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
}

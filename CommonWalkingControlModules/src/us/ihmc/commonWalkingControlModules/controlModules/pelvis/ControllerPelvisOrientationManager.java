package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ControllerPelvisOrientationManager extends PelvisOrientationControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameQuaternion desiredPelvisOrientation = new YoFrameQuaternion("desiredPelvis", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularVelocity = new YoFrameVector("desiredPelvisAngularVelocity", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", worldFrame, registry);

   private final DoubleYoVariable initialPelvisOrientationTime = new DoubleYoVariable("initialPelvisOrientationTime", registry);
   private final YoFrameQuaternion initialPelvisOrientation = new YoFrameQuaternion("initialPelvis", worldFrame, registry);
   private final YoFrameQuaternion finalPelvisOrientation = new YoFrameQuaternion("finalPelvis", worldFrame, registry);
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;

   private final DoubleYoVariable initialPelvisOrientationOffsetTime = new DoubleYoVariable("initialPelvisOrientationOffsetTime", registry);

   private final YoFrameQuaternion desiredPelvisOrientationWithOffset = new YoFrameQuaternion("desiredPelvisOrientationWithOffset", worldFrame, registry);

   private final DoubleYoVariable yoTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final YoFrameVector yoPelvisAngularWeight = new YoFrameVector("pelvisWeight", null, registry);
   private final Vector3D pelvisAngularWeight = new Vector3D();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<ReferenceFrame> soleFrames;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isPelvisOrientationOffsetTrajectoryStopped", registry);

   private final YoOrientationPIDGainsInterface gains;

   private final BooleanYoVariable followPelvisYawSineWave = new BooleanYoVariable("followPelvisYawSineWave", registry);
   private final DoubleYoVariable pelvisYawSineFrequence = new DoubleYoVariable("pelvisYawSineFrequence", registry);
   private final DoubleYoVariable pelvisYawSineMagnitude = new DoubleYoVariable("pelvisYawSineMagnitude", registry);

   private final FrameOrientation upcomingFootstepOrientation = new FrameOrientation();

   public ControllerPelvisOrientationManager(YoOrientationPIDGainsInterface gains, HighLevelHumanoidControllerToolbox controllerToolbox,
         YoVariableRegistry parentRegistry)
   {
      super(PelvisOrientationControlMode.WALKING_CONTROLLER);

      yoTime = controllerToolbox.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      soleFrames = referenceFrames.getSoleFrames();
      pelvisFrame = referenceFrames.getPelvisFrame();

      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);
      pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(ankleZUpFrames.get(robotSide));

      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();
      yoPelvisAngularWeight.set(SolverWeightLevels.PELVIS_WEIGHT, SolverWeightLevels.PELVIS_WEIGHT, SolverWeightLevels.PELVIS_WEIGHT);
      yoPelvisAngularWeight.get(pelvisAngularWeight);
      orientationFeedbackControlCommand.set(elevator, pelvis);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);

      desiredPelvisFrame = new ReferenceFrame("desiredPelvisFrame", worldFrame)
      {
         private static final long serialVersionUID = -1472151257649344278L;

         private final Quaternion rotationToParent = new Quaternion();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            pelvisFrame.getTransformToDesiredFrame(transformToParent, parentFrame);
            desiredPelvisOrientation.get(rotationToParent);
            transformToParent.setRotation(rotationToParent);
         }
      };

      pelvisOrientationOffsetTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvisOffset", false, desiredPelvisFrame, registry);

      pelvisYawSineFrequence.set(1.0);
      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      yoPelvisAngularWeight.set(weight, weight, weight);
   }

   public void setWeights(Vector3D weight)
   {
      yoPelvisAngularWeight.set(weight);
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

   @Override
   public void doAction()
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

      if (followPelvisYawSineWave.getBooleanValue())
      {
         double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(yoTime.getDoubleValue() * pelvisYawSineFrequence.getDoubleValue() * 2.0 * Math.PI);
         tempOrientation.setIncludingFrame(midFeetZUpFrame, yaw, 0.0, 0.0);

         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);
         tempAngularAcceleration.setToZero(worldFrame);

         desiredPelvisOrientation.set(tempOrientation);
         desiredPelvisAngularVelocity.set(tempAngularVelocity);
         desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
         desiredPelvisFrame.update();
      }
      else if (isTrajectoryStopped.getBooleanValue())
      {
         pelvisOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
         tempAngularVelocity.setToZero();
         tempAngularAcceleration.setToZero();
      }
      else
      {
         double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
         pelvisOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
         pelvisOrientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
      }

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
      yoPelvisAngularWeight.get(pelvisAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      pelvisOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
      tempOrientation.changeFrame(desiredPelvisFrame);
      tempAngularVelocity.setToZero(desiredPelvisFrame);

      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();

      isTrajectoryStopped.set(false);
   }

   public void setOffset(FrameOrientation offset)
   {
      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setToHoldCurrentInWorldFrame()
   {
      setToHoldCurrent(worldFrame);
   }

   public void setToHoldCurrent(ReferenceFrame trajectoryFrame)
   {
      tempOrientation.setToZero(pelvisFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      resetOrientationOffset();
      initialize(trajectoryFrame);
   }

   public void centerInMidFeetZUpFrame(double trajectoryTime)
   {
      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.setAndMatchFrame(tempOrientation);

      tempOrientation.setToZero(midFeetZUpFrame);
      finalPelvisOrientation.setAndMatchFrame(tempOrientation);

      setTrajectoryTime(trajectoryTime);
      initialize(midFeetZUpFrame);
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

   public void setWithUpcomingFootstep(Footstep upcomingFootstep)
   {
      RobotSide upcomingFootstepSide = upcomingFootstep.getRobotSide();

      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      upcomingFootstep.getOrientation(upcomingFootstepOrientation);
      upcomingFootstepOrientation.changeFrame(worldFrame);
      tempOrientation.setToZero(soleFrames.get(upcomingFootstepSide.getOppositeSide()));
      tempOrientation.changeFrame(worldFrame);

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(upcomingFootstepOrientation.getYaw(), tempOrientation.getYaw());
      finalPelvisOrientation.set(finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(worldFrame);
   }

   @Override
   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }

   @Override
   public void getCurrentDesiredOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(desiredPelvisOrientationWithOffset.getFrameOrientation());
   }
}

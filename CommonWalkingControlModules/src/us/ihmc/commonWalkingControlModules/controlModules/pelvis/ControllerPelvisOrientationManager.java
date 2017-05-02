package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ControllerPelvisOrientationManager extends PelvisOrientationControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FrameOrientation desiredPelvisOrientation = new FrameOrientation();
   private final FrameVector desiredPelvisAngularVelocity = new FrameVector();
   private final FrameVector desiredPelvisAngularAcceleration = new FrameVector();
   private final FrameOrientation desiredPelvisOrientationWithOffset = new FrameOrientation();

   private final FrameOrientation initialPelvisOrientation = new FrameOrientation();
   private final FrameOrientation finalPelvisOrientation = new FrameOrientation();

   private final SimpleOrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;

   private final DoubleYoVariable initialPelvisOrientationTime = new DoubleYoVariable("initialPelvisOrientationTime", registry);
   private final DoubleYoVariable initialPelvisOrientationOffsetTime = new DoubleYoVariable("initialPelvisOrientationOffsetTime", registry);
   private final DoubleYoVariable yoTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final YoFrameVector yoPelvisAngularWeight = new YoFrameVector("pelvisWeight", null, registry);
   private final Vector3D pelvisAngularWeight = new Vector3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();

   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final ReferenceFrame midFeetZUpGroundFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final YoOrientationPIDGainsInterface gains;

   public ControllerPelvisOrientationManager(YoOrientationPIDGainsInterface gains, HighLevelHumanoidControllerToolbox controllerToolbox,
         YoVariableRegistry parentRegistry)
   {
      super(PelvisOrientationControlMode.WALKING_CONTROLLER);

      yoTime = controllerToolbox.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      midFeetZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();
      pelvisFrame = referenceFrames.getPelvisFrame();

      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);
      pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpGroundFrame);
      for (RobotSide robotSide : RobotSide.values)
         pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(soleZUpFrames.get(robotSide));

      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();
      yoPelvisAngularWeight.set(SolverWeightLevels.PELVIS_WEIGHT, SolverWeightLevels.PELVIS_WEIGHT, SolverWeightLevels.PELVIS_WEIGHT);
      yoPelvisAngularWeight.get(pelvisAngularWeight);
      orientationFeedbackControlCommand.set(elevator, pelvis);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);
      selectionMatrix.resetSelection();

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
      pelvisOrientationTrajectoryGenerator.setInitialOrientation(initialPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.setFinalOrientation(finalPelvisOrientation);

      pelvisOrientationTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.setIncludingFrame(tempAngularVelocity);
      desiredPelvisAngularAcceleration.setIncludingFrame(tempAngularAcceleration);
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

      desiredPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.setIncludingFrame(tempAngularVelocity);
      desiredPelvisAngularAcceleration.setIncludingFrame(tempAngularAcceleration);
      desiredPelvisFrame.update();

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      pelvisOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      pelvisOrientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientationWithOffset.setIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      orientationFeedbackControlCommand.set(desiredPelvisOrientationWithOffset, desiredPelvisAngularVelocity, desiredPelvisAngularAcceleration);
      yoPelvisAngularWeight.get(pelvisAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);
      orientationFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
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
      initialPelvisOrientation.setIncludingFrame(tempOrientation);
      finalPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisOrientation.setIncludingFrame(tempOrientation);

      resetOrientationOffset();
      initialize(trajectoryFrame);
   }

   public void centerInMidFeetZUpFrame(double trajectoryTime)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      finalPelvisOrientation.setToZero(midFeetZUpGroundFrame);
      setTrajectoryTime(trajectoryTime);
      initialize(midFeetZUpGroundFrame);
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      setToHoldCurrentDesired(midFeetZUpGroundFrame);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      setToHoldCurrentDesired(soleZUpFrames.get(supportSide));
   }

   public void setToHoldCurrentDesired(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      finalPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      initialize(desiredTrajectoryFrame);
   }

   /** Go instantly to zero, no smooth interpolation. */
   public void setToZeroInMidFeetZUpFrame()
   {
      tempOrientation.setToZero(midFeetZUpGroundFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.setIncludingFrame(tempOrientation);
      finalPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisOrientation.setIncludingFrame(tempOrientation);

      initialize(midFeetZUpGroundFrame);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);

      ReferenceFrame otherAnkleZUpFrame = soleZUpFrames.get(supportSide.getOppositeSide());
      ReferenceFrame supportAnkleZUpFrame = soleZUpFrames.get(supportSide);

      tempOrientation.setToZero(otherAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawOtherFoot = tempOrientation.getYaw();

      tempOrientation.setToZero(supportAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawOtherFoot, yawSupportFoot);
      finalPelvisOrientation.setIncludingFrame(worldFrame, finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(supportAnkleZUpFrame);
   }

   public void setWithUpcomingFootstep(Footstep upcomingFootstep)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);

      RobotSide upcomingFootstepSide = upcomingFootstep.getRobotSide();
      ReferenceFrame supportSoleFrame = soleZUpFrames.get(upcomingFootstepSide.getOppositeSide());

      upcomingFootstep.getOrientation(tempOrientation);
      tempOrientation.changeFrame(worldFrame);
      double yawFootstep = tempOrientation.getYaw();

      tempOrientation.setToZero(supportSoleFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawFootstep, yawSupportFoot);
      finalPelvisOrientation.setIncludingFrame(worldFrame, finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(worldFrame);
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }

   @Override
   public void getCurrentDesiredOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(desiredPelvisOrientationWithOffset);
   }
}
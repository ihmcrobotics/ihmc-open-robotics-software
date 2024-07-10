package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.referenceFrames.WalkingTrajectoryPath;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerPelvisOrientationManager implements PelvisOrientationControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameQuaternion desiredPelvisOrientation = new FrameQuaternion();
   private final FrameVector3D desiredPelvisAngularVelocity = new FrameVector3D();
   private final FrameVector3D desiredPelvisAngularAcceleration = new FrameVector3D();
   private final FrameQuaternion desiredPelvisOrientationWithOffset = new FrameQuaternion();

   private final SimpleOrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;

   private final YoDouble initialPelvisOrientationOffsetTime = new YoDouble("initialPelvisOrientationOffsetTime", registry);
   private final YoDouble yoTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private Vector3DReadOnly pelvisAngularWeight = null;
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FrameVector3D tempAngularAcceleration = new FrameVector3D();

   private final WalkingTrajectoryPath walkingTrajectoryPath;
   private final ReferenceFrame pelvisFrame;
   private final MovingReferenceFrame walkingPathFrame;

   private final PID3DGainsReadOnly gains;

   public ControllerPelvisOrientationManager(PID3DGainsReadOnly gains, HighLevelHumanoidControllerToolbox controllerToolbox, YoRegistry parentRegistry)
   {
      yoTime = controllerToolbox.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      pelvisFrame = referenceFrames.getPelvisFrame();

      walkingTrajectoryPath = controllerToolbox.getWalkingTrajectoryPath();

      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      orientationFeedbackControlCommand.set(elevator, pelvis);
      selectionMatrix.resetSelection();

      walkingPathFrame = walkingTrajectoryPath.getWalkingTrajectoryPathFrame();

      pelvisOrientationOffsetTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvisOffset", false, walkingPathFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly pelvisAngularWeight)
   {
      this.pelvisAngularWeight = pelvisAngularWeight;
   }

   private final Twist pelvisTwist = new Twist();

   @Override
   public void doAction(double timeInState)
   {
      desiredPelvisOrientation.setToZero(walkingPathFrame);
      walkingPathFrame.getTwistRelativeToOther(worldFrame, pelvisTwist);
      desiredPelvisAngularVelocity.setMatchingFrame(pelvisTwist.getAngularPart());
      desiredPelvisAngularAcceleration.setMatchingFrame(walkingTrajectoryPath.getWalkingTrajectoryPathAcceleration().getAngularPart());

      desiredPelvisOrientation.changeFrame(worldFrame);
      desiredPelvisAngularVelocity.changeFrame(worldFrame);
      desiredPelvisAngularAcceleration.changeFrame(worldFrame);

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      pelvisOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      pelvisOrientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientationWithOffset.setIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      orientationFeedbackControlCommand.setInverseDynamics(desiredPelvisOrientationWithOffset, desiredPelvisAngularVelocity, desiredPelvisAngularAcceleration);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);
      orientationFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
   }

   @Override
   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      goToHomeFromOffset(pelvisOrientationOffsetTrajectoryGenerator.getOrientation(), trajectoryTime);
   }

   public void goToHomeFromOffset(FrameQuaternionReadOnly offset, double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(walkingPathFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      tempOrientation.setToZero(walkingPathFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setOffset(FrameQuaternionReadOnly offset)
   {
      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(walkingPathFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(walkingPathFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setToHoldCurrentInWorldFrame()
   {
      desiredPelvisOrientation.setToZero(walkingPathFrame);
      desiredPelvisOrientationWithOffset.setToZero(pelvisFrame);
      desiredPelvisOrientationWithOffset.changeFrame(walkingPathFrame);

      tempOrientation.setReferenceFrame(walkingPathFrame);
      tempOrientation.difference(desiredPelvisOrientationWithOffset, desiredPelvisOrientation);

      setOffset(tempOrientation);
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
}
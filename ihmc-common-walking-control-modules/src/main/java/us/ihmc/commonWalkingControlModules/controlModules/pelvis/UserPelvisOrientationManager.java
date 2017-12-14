package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class UserPelvisOrientationManager extends PelvisOrientationControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FramePose tempPose = new FramePose();
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final ReferenceFrame baseFrame;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAcceleration = new FrameVector3D();

   private final FramePose homePose;

   public UserPelvisOrientationManager(PID3DGainsReadOnly gains, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      super(PelvisOrientationControlMode.USER);

      RigidBody pelvis = controllerToolbox.getFullRobotModel().getPelvis();
      RigidBody elevator = controllerToolbox.getFullRobotModel().getElevator();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      ReferenceFrame pelvisFixedFrame = pelvis.getBodyFixedFrame();
      baseFrame = controllerToolbox.getReferenceFrames().getMidFootZUpGroundFrame();
      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      taskspaceControlState = new RigidBodyTaskspaceControlState("Orientation", pelvis, elevator, elevator, trajectoryFrames, pelvisFixedFrame, baseFrame, yoTime, null, graphicsListRegistry, registry);
      taskspaceControlState.setGains(gains, null);

      orientationFeedbackControlCommand.set(elevator, pelvis);
      orientationFeedbackControlCommand.setPrimaryBase(elevator);

      homePose = new FramePose(baseFrame);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly angularWeight)
   {
      taskspaceControlState.setWeights(angularWeight, null);
   }

   @Override
   public void doAction()
   {
      taskspaceControlState.doAction();
   }

   public boolean handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command, FrameQuaternion initialOrientation)
   {
      tempPose.setToNaN(initialOrientation.getReferenceFrame());
      tempPose.setOrientation(initialOrientation);
      return taskspaceControlState.handleOrientationTrajectoryCommand(command, tempPose);
   }

   public ReferenceFrame getControlFrame()
   {
      return taskspaceControlState.getControlFrame();
   }

   @Override
   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      taskspaceControlState.goToPoseFromCurrent(homePose, trajectoryTime);
   }

   @Override
   public void getCurrentDesiredOrientation(FrameQuaternion desiredOrientation)
   {
      taskspaceControlState.getDesiredPose(tempPose);
      tempPose.getOrientationIncludingFrame(desiredOrientation);
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      SpatialFeedbackControlCommand spatialFeedbackControlCommand = taskspaceControlState.getSpatialFeedbackControlCommand();
      orientationFeedbackControlCommand.setGains(spatialFeedbackControlCommand.getGains().getOrientationGains());
      orientationFeedbackControlCommand.getSpatialAccelerationCommand().set(spatialFeedbackControlCommand.getSpatialAccelerationCommand());
      spatialFeedbackControlCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      orientationFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      return orientationFeedbackControlCommand;
   }
}

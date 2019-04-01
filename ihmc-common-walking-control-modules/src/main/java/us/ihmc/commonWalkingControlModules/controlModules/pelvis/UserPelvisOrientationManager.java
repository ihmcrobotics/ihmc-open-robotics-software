package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyOrientationController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class UserPelvisOrientationManager implements PelvisOrientationControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final RigidBodyOrientationController orientationController;
   private final ReferenceFrame baseFrame;
   private final FrameQuaternion homeOrientation;

   public UserPelvisOrientationManager(PID3DGainsReadOnly gains, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      RigidBodyBasics pelvis = controllerToolbox.getFullRobotModel().getPelvis();
      RigidBodyBasics elevator = controllerToolbox.getFullRobotModel().getElevator();
      baseFrame = controllerToolbox.getReferenceFrames().getMidFootZUpGroundFrame();
      YoDouble yoTime = controllerToolbox.getYoTime();

      orientationController = new RigidBodyOrientationController(pelvis, elevator, elevator, baseFrame, yoTime, null, registry);
      orientationController.setGains(gains);

      homeOrientation = new FrameQuaternion(baseFrame);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly angularWeight)
   {
      orientationController.setWeights(angularWeight);
   }

   @Override
   public void doAction(double timeInState)
   {
      orientationController.doAction(timeInState);
   }

   public boolean handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command)
   {
      SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
      so3Trajectory.setSequenceId(command.getSequenceId());
      return orientationController.handleTrajectoryCommand(so3Trajectory);
   }

   @Override
   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      orientationController.goToOrientation(homeOrientation, trajectoryTime);
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationController.getFeedbackControlCommand();
   }

   @Override
   public void onEntry()
   {
      orientationController.onEntry();
   }

   @Override
   public void onExit()
   {
      orientationController.onExit();
   }

   public FrameQuaternionReadOnly getDesiredOrientation()
   {
      return orientationController.getDesiredOrientation();
   }

   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return orientationController.pollStatusToReport();
   }
}

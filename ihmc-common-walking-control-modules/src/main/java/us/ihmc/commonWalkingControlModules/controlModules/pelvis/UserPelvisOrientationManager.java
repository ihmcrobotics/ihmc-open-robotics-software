package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyOrientationController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
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
      RigidBody pelvis = controllerToolbox.getFullRobotModel().getPelvis();
      RigidBody elevator = controllerToolbox.getFullRobotModel().getElevator();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      baseFrame = controllerToolbox.getReferenceFrames().getMidFootZUpGroundFrame();
      YoDouble yoTime = controllerToolbox.getYoTime();

      orientationController = new RigidBodyOrientationController(pelvis, elevator, elevator, trajectoryFrames, baseFrame, yoTime, null, registry);
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
      return orientationController.handleTrajectoryCommand(command.getSO3Trajectory());
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
}

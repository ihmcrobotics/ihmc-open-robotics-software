package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerCommandInputManager;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class UserDesiredControllerCommandGenerators
{
   public UserDesiredControllerCommandGenerators(ControllerCommandInputManager controllerCommandInputManager, FullHumanoidRobotModel fullRobotModel, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      new UserDesiredFootPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredHandPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredChestOrientationControllerCommandGenerator(controllerCommandInputManager, defaultTrajectoryTime, parentRegistry);
   }
}

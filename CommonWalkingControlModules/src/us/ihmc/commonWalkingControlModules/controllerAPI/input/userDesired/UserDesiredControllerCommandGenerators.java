package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;

public class UserDesiredControllerCommandGenerators
{
   public UserDesiredControllerCommandGenerators(CommandInputManager controllerCommandInputManager, FullHumanoidRobotModel fullRobotModel,
         SideDependentList<ContactableFoot> bipedFeet, WalkingControllerParameters walkingControllerParameters, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      new UserDesiredFootPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredHandPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredChestOrientationControllerCommandGenerator(controllerCommandInputManager, defaultTrajectoryTime, parentRegistry);
      new UserDesiredPelvisHeightControllerCommandGenerators(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredFootstepDataMessageGenerator(controllerCommandInputManager, bipedFeet, walkingControllerParameters, parentRegistry);
   }
}

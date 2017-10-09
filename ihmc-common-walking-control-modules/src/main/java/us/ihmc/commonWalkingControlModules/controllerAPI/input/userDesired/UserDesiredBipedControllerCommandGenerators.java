package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;

public class UserDesiredBipedControllerCommandGenerators
{
   public UserDesiredBipedControllerCommandGenerators(CommandInputManager controllerCommandInputManager, FullHumanoidRobotModel fullRobotModel,
         SideDependentList<ContactableFoot> bipedFeet, WalkingControllerParameters walkingControllerParameters, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      new UserDesiredFootPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredFootstepDataMessageGenerator(controllerCommandInputManager, bipedFeet, walkingControllerParameters, parentRegistry);
      new UserDesiredPelvisHeightControllerCommandGenerators(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
   }
}

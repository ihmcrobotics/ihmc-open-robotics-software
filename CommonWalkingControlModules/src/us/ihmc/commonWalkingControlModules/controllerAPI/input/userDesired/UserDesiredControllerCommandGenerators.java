package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class UserDesiredControllerCommandGenerators
{
   public UserDesiredControllerCommandGenerators(CommandInputManager controllerCommandInputManager, FullHumanoidRobotModel fullRobotModel,
         CommonHumanoidReferenceFrames commonHumanoidReferenceFrames, SideDependentList<ContactableFoot> bipedFeet, 
         WalkingControllerParameters walkingControllerParameters, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      new UserDesiredFootPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredHandPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredChestOrientationControllerCommandGenerator(controllerCommandInputManager, defaultTrajectoryTime, parentRegistry);
      new UserDesiredPelvisHeightControllerCommandGenerators(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredFootstepDataMessageGenerator(controllerCommandInputManager, bipedFeet, walkingControllerParameters, parentRegistry);
      new UserDesiredPelvisPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, commonHumanoidReferenceFrames, defaultTrajectoryTime, parentRegistry);
   }
}

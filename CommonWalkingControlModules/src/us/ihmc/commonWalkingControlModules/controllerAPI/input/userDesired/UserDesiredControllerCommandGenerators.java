package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.CommandInputManager;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class UserDesiredControllerCommandGenerators
{
   public final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   public UserDesiredControllerCommandGenerators(CommandInputManager controllerCommandInputManager, FullHumanoidRobotModel fullRobotModel,
         SideDependentList<ContactableFoot> bipedFeet, WalkingControllerParameters walkingControllerParameters, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      new UserDesiredFootPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredHandPoseControllerCommandGenerator(controllerCommandInputManager, fullRobotModel, defaultTrajectoryTime, parentRegistry);
      new UserDesiredChestOrientationControllerCommandGenerator(controllerCommandInputManager, defaultTrajectoryTime, parentRegistry);
      UserDesiredFootstepDataMessageGenerator footstepGenerator = new UserDesiredFootstepDataMessageGenerator(bipedFeet, controllerCommandInputManager, walkingControllerParameters, parentRegistry);

      updatables.add(footstepGenerator);
   }

   public ArrayList<Updatable> getUpdatables()
   {
      return updatables;
   }
}

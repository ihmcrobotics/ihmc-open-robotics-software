package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class HumanoidKinematicsToolboxController extends KinematicsToolboxController
{
   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoVariableRegistry parentRegistry)
   {
      super(commandInputManager, statusOutputManager, desiredFullRobotModel, yoGraphicsListRegistry, parentRegistry);
   }
}

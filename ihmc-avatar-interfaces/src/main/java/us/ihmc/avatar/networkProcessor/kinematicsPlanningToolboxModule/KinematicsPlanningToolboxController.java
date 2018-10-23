package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class KinematicsPlanningToolboxController extends ToolboxController
{

   public KinematicsPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
   }

   @Override
   protected boolean initialize()
   {
      return false;
   }

   @Override
   protected void updateInternal() throws Exception
   {

   }

   @Override
   protected boolean isDone()
   {
      return false;
   }
}

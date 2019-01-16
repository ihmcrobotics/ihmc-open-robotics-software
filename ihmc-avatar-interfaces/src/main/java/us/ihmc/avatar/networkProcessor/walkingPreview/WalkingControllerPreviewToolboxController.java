package us.ihmc.avatar.networkProcessor.walkingPreview;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WalkingControllerPreviewToolboxController extends ToolboxController
{
   public WalkingControllerPreviewToolboxController(DRCRobotModel robotModel, CommandInputManager commandInputManager,
                                                    StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                    YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
   }

   @Override
   public boolean initialize()
   {
      return false;
   }

   @Override
   public void updateInternal() throws Exception
   {
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}

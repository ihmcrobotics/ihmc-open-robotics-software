package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface HighLevelHumanoidControllerPlugin extends Updatable
{
   default YoRegistry getRegistry()
   {
      return null;
   }

   default YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}

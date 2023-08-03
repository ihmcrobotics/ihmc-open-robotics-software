package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface HighLevelHumanoidControllerPlugin extends Updatable, SCS2YoGraphicHolder
{
   default YoRegistry getRegistry()
   {
      return null;
   }

   @Override
   default YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}

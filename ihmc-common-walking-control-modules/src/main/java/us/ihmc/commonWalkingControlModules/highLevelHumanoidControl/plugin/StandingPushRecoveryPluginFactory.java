package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class StandingPushRecoveryPluginFactory implements HighLevelHumanoidControllerPluginFactory
{
   @Override
   public HighLevelHumanoidControllerPlugin buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper, DoubleProvider updateDT)
   {
      return new StandingPushRecoveryPlugin(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());
   }
}

package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;

public class StandingPushRecoveryPluginFactory implements HighLevelHumanoidControllerPluginFactory
{
   @Override
   public HighLevelHumanoidControllerPlugin buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      return new StandingPushRecoveryPlugin(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());
   }
}

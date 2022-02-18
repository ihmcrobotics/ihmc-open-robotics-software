package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public class DefaultPushRecoveryControllerParameters implements PushRecoveryControllerParameters
{
   public DefaultPushRecoveryControllerParameters()
   {

   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return null;
   }

   @Override
   public double getMaxStepLength()
   {
      return 1.0;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.1;
   }
}

package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public abstract class JumpControllerParameters
{
   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();

}

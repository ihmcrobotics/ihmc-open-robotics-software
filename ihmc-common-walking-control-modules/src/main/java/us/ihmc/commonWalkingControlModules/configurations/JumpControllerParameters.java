package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;

public abstract class JumpControllerParameters
{
   public abstract ControllerCoreOptimizationSettings getMomentumOptimizationSettings();

}

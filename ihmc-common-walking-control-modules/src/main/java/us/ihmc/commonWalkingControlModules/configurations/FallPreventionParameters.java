package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public interface FallPreventionParameters
{
   JointOfflineParameters getJointOfflineParameters();

   MomentumOptimizationSettings getFallPreventionMomentumOptimizationSettings();
}

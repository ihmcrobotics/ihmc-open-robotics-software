package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public abstract class JumpControllerParameters extends AbstractHighLevelControllerParameters
{
   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();
   public abstract double getMinimumPrincipalInertia();
   public abstract double getMaximumPrincipalInertia();
   public abstract double getAngularVelocityRegularizationWeights();
   public abstract double getMaximumInertiaRateOfChange();
}

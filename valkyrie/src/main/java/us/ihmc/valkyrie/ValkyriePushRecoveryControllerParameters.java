package us.ihmc.valkyrie;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieMomentumOptimizationSettings;

public class ValkyriePushRecoveryControllerParameters implements PushRecoveryControllerParameters
{
   private final ValkyrieMomentumOptimizationSettings momentumOptimizationSettings;

   public ValkyriePushRecoveryControllerParameters(ValkyrieJointMap jointMap)
   {
      momentumOptimizationSettings = new ValkyrieMomentumOptimizationSettings(jointMap);
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   @Override
   public double getRecoveryTransferDuration()
   {
      return 0.15;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.7;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15;
   }
}

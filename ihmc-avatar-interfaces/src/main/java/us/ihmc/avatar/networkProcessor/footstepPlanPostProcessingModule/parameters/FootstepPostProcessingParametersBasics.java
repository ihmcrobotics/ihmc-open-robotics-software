package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters;

import us.ihmc.tools.property.StoredPropertySetBasics;

public interface FootstepPostProcessingParametersBasics extends FootstepPostProcessingParametersReadOnly, StoredPropertySetBasics
{
   default void set(FootstepPostProcessingParametersReadOnly footstepPostProcessingParameters)
   {
      setAll(footstepPostProcessingParameters.getAll());
   }

   default void setSplitFractionProcessingEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.splitFractionProcessingEnabled, enabled);
   }

   default void setSwingOverRegionsEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.swingOverRegionsEnabled, enabled);
   }

   default void setStepHeightForLargeStepDown(double height)
   {
      set(FootstepPostProcessingKeys.stepHeightForLargeStepDown, height);
   }

   default void setLargestStepDownHeight(double height)
   {
      set(FootstepPostProcessingKeys.largestStepDownHeight, height);
   }

   default void setTransferSplitFractionAtFullDepth(double splitFraction)
   {
      set(FootstepPostProcessingKeys.transferSplitFractionAtFullDepth, splitFraction);
   }

   default void setTransferWeightDistributionAtFullDepth(double weightDistribution)
   {
      set(FootstepPostProcessingKeys.transferWeightDistributionAtFullDepth, weightDistribution);
   }
}

package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

public interface FootstepPostProcessingParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean splitFractionProcessingEnabled()
   {
      return get(FootstepPostProcessingKeys.splitFractionProcessingEnabled);
   }

   default double getStepHeightForLargeStepDown()
   {
      return get(FootstepPostProcessingKeys.stepHeightForLargeStepDown);
   }

   default double getLargestStepDownHeight()
   {
      return get(FootstepPostProcessingKeys.largestStepDownHeight);
   }

   default double getTransferSplitFractionAtFullDepth()
   {
      return get(FootstepPostProcessingKeys.transferSplitFractionAtFullDepth);
   }

   default double getTransferWeightDistributionAtFullDepth()
   {
      return get(FootstepPostProcessingKeys.transferWeightDistributionAtFullDepth);
   }
}

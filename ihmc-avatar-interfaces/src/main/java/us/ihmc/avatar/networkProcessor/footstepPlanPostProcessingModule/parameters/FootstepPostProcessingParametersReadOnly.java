package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

public interface FootstepPostProcessingParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean splitFractionProcessingEnabled()
   {
      return get(FootstepPostProcessingKeys.splitFractionProcessingEnabled);
   }
}

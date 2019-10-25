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
}

package us.ihmc.footstepPlanning.postProcessing.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoVariablesForFootstepPostProcessingParameters extends YoVariablesForStoredProperties
{
   public YoVariablesForFootstepPostProcessingParameters(YoVariableRegistry parentRegistry, FootstepPostProcessingParametersBasics defaults)
   {
      super(defaults, FootstepPostProcessingKeys.keys, YoVariablesForFootstepPostProcessingParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }
}

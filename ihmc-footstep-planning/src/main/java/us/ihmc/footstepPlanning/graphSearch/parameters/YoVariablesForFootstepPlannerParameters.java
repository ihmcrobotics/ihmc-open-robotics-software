package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoVariablesForFootstepPlannerParameters extends YoVariablesForStoredProperties
{
   public YoVariablesForFootstepPlannerParameters(YoRegistry parentRegistry, FootstepPlannerParametersBasics defaults)
   {
      super(defaults, FootstepPlannerParameterKeys.keys, YoVariablesForFootstepPlannerParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }
}

package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoVariablesForFootstepPlannerParameters extends YoVariablesForStoredProperties
{
   public YoVariablesForFootstepPlannerParameters(YoRegistry parentRegistry, DefaultFootstepPlannerParametersBasics defaults)
   {
      super(defaults, DefaultFootstepPlannerParameters.keys, YoVariablesForFootstepPlannerParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }
}

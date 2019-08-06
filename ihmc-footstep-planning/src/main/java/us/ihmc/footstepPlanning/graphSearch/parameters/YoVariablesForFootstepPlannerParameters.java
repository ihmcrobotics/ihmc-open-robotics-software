package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoVariablesForFootstepPlannerParameters extends YoVariablesForStoredProperties
{
   public YoVariablesForFootstepPlannerParameters(YoVariableRegistry parentRegistry, FootstepPlannerParametersReadOnly defaults)
   {
      super(defaults.getStoredPropertySet(), FootstepPlannerParameterKeys.keys, YoVariablesForFootstepPlannerParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }
}

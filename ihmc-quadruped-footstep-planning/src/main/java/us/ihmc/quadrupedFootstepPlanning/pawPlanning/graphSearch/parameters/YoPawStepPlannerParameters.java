package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoPawStepPlannerParameters extends YoVariablesForStoredProperties
{
   public YoPawStepPlannerParameters(PawStepPlannerParametersBasics parameters, YoVariableRegistry parentRegistry)
   {
      super(parameters, PawStepPlannerParameterKeys.keys, YoPawStepPlannerParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }


}

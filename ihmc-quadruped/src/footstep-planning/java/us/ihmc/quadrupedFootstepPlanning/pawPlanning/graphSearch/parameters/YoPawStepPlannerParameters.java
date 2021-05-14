package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPawStepPlannerParameters extends YoVariablesForStoredProperties
{
   public YoPawStepPlannerParameters(PawStepPlannerParametersBasics parameters, YoRegistry parentRegistry)
   {
      super(parameters, PawStepPlannerParameterKeys.keys, YoPawStepPlannerParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }


}

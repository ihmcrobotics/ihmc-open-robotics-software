package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoPawPlannerParameters extends YoVariablesForStoredProperties
{
   public YoPawPlannerParameters(PawPlannerParametersBasics parameters, YoVariableRegistry parentRegistry)
   {
      super(parameters, PawPlannerParameterKeys.keys, YoPawPlannerParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }


}

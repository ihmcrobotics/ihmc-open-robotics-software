package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import us.ihmc.tools.property.YoVariablesForStoredProperties;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoVisibilityGraphParameters extends YoVariablesForStoredProperties
{
   public YoVisibilityGraphParameters(YoVariableRegistry parentRegistry, VisibilityGraphsParametersBasics defaults)
   {
      super(defaults, VisibilityGraphParametersKeys.keys, YoVisibilityGraphParameters.class.getSimpleName());
      parentRegistry.addChild(getRegistry());
   }
}

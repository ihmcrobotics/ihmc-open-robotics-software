package us.ihmc.robotics.parameterGui.tree;

import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterTreeValue
{
   private final String name;
   private final boolean isRegistry;
   private final String description;

   public ParameterTreeValue(Registry registry)
   {
      name = registry.getName();
      isRegistry = true;
      description = null;
   }

   public ParameterTreeValue(Parameter parameter)
   {
      name = parameter.getName();
      isRegistry = false;
      description = parameter.getDescription();
   }

   public boolean isRegistry()
   {
      return isRegistry;
   }

   public String getDescription()
   {
      return description;
   }

   public String getName()
   {
      return name;
   }
}

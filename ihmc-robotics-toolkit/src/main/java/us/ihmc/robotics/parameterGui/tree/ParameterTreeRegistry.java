package us.ihmc.robotics.parameterGui.tree;

import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterTreeRegistry extends ParameterTreeValue
{
   private final Registry registry;

   public ParameterTreeRegistry(Registry registry)
   {
      this.registry = registry;
   }

   @Override
   public boolean isRegistry()
   {
      return true;
   }

   @Override
   public String getDescription()
   {
      return "Parameter Registry";
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }
}

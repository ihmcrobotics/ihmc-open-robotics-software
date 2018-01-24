package us.ihmc.robotics.parameterGui.tree;

import us.ihmc.robotics.parameterGui.GuiRegistry;

public class ParameterTreeRegistry extends ParameterTreeValue
{
   private final GuiRegistry registry;

   public ParameterTreeRegistry(GuiRegistry registry)
   {
      this.registry = registry;
   }

   @Override
   public boolean isRegistry()
   {
      return true;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }
}

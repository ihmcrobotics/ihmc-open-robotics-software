package us.ihmc.robotics.parameterGui.tree;

import javafx.scene.Node;
import javafx.scene.text.Text;
import us.ihmc.robotics.parameterGui.GuiRegistry;

public class ParameterTreeRegistry implements ParameterTreeValue
{
   private final GuiRegistry registry;
   private Node node;

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

   @Override
   public Node getOrCreateNode()
   {
      if (node == null)
      {
         node = new Text(registry.getName());
      }
      return node;
   }
}

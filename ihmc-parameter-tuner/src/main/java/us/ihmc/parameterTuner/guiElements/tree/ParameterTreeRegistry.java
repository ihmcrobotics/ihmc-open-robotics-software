package us.ihmc.parameterTuner.guiElements.tree;

import javafx.scene.Node;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;

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

   @Override
   public GuiParameter getParameter()
   {
      throw new RuntimeException("Is a registry!");
   }
}

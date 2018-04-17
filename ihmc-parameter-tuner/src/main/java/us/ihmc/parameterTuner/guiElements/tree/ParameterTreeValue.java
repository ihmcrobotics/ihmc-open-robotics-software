package us.ihmc.parameterTuner.guiElements.tree;

import javafx.scene.Node;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public interface ParameterTreeValue
{
   public boolean isRegistry();

   public String getName();

   public Node getOrCreateNode();

   public GuiParameter getParameter();
}

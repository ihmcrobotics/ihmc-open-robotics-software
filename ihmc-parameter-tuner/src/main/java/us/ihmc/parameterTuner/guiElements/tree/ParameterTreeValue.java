package us.ihmc.parameterTuner.guiElements.tree;

import javafx.scene.Node;

public interface ParameterTreeValue
{
   public boolean isRegistry();

   public String getName();

   public Node getOrCreateNode();
}

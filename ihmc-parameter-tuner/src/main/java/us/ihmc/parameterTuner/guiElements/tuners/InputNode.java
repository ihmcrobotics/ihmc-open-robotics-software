package us.ihmc.parameterTuner.guiElements.tuners;

import javafx.scene.Node;

public interface InputNode
{
   /**
    * Creates a linked duplicate of the main input node.
    */
   public abstract Node getSimpleInputNode();
}

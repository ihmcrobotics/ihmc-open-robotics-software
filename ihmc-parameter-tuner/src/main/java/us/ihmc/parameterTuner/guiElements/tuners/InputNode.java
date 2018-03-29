package us.ihmc.parameterTuner.guiElements.tuners;

import javafx.scene.Node;

public interface InputNode
{
   /**
    * Creates a linked duplicate of the main input node.
    */
   public abstract Node getSimpleInputNode(double width, double height);

   /**
    * Retries the full tuning node with all additional options such as min and max value.
    */
   public abstract Node getFullInputNode();
}

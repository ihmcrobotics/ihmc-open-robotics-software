package us.ihmc.parameterTuner.guiElements.tuners;

import javafx.scene.Node;

public interface InputNode
{
   /**
    * Creates a linked duplicate of the main input node.
    */
   Node getSimpleInputNode(double width, double height);

   /**
    * Retries the full tuning node with all additional options such as min and max value.
    */
   Node getFullInputNode();

   /**
    * Sets the value of the parameter associated with this input node based on the provided
    * percent value.
    */
   void setValueFromPercent(double percent);

   /**
    * Gets the value of the parameter as a percent value.
    */
   double getValuePercent();
}

package us.ihmc.parameterTuner.guiElements.tree;

import javafx.scene.control.TreeCell;

public class ParameterTreeCell extends TreeCell<ParameterTreeValue>
{
   @Override
   protected void updateItem(ParameterTreeValue value, boolean empty)
   {
      super.updateItem(value, empty);
      setText(null);

      if (empty || value == null)
      {
         setGraphic(null);
      }
      else
      {
         setGraphic(value.getOrCreateNode());
      }
   }
}

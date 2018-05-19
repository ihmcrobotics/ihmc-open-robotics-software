package us.ihmc.parameterTuner.guiElements.tree;

import javafx.css.PseudoClass;
import javafx.scene.control.TreeCell;

public class ParameterTreeCell extends TreeCell<ParameterTreeValue>
{
   private final PseudoClass isRegistry = PseudoClass.getPseudoClass("is-registry");

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
         pseudoClassStateChanged(isRegistry, value.isRegistry());
      }
   }
}

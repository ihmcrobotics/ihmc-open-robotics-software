package us.ihmc.robotics.parameterGui.tree;

import javafx.css.PseudoClass;
import javafx.scene.control.TreeCell;

public class ParameterTreeCell extends TreeCell<ParameterTreeValue>
{
   private final PseudoClass isRegistry = PseudoClass.getPseudoClass("is-registry");

   @Override
   protected void updateItem(ParameterTreeValue value, boolean empty)
   {
      super.updateItem(value, empty);

      if (empty || value == null)
      {
         setText(null);
      }
      else
      {
         setText(value.getName());
         pseudoClassStateChanged(isRegistry, value.isRegistry());
      }
   }
}

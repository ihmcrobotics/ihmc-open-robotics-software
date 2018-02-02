package us.ihmc.parameterTuner.guiElements.tree;

import javafx.scene.control.TreeItem;

public class ParameterTreeItem extends TreeItem<ParameterTreeValue>
{
   public ParameterTreeItem(ParameterTreeValue parameterTreeValue)
   {
      super(parameterTreeValue);
      expandedProperty().addListener((observable, oldValue, newValue) -> expandChildrenIfEmpty());
   }

   public void expandChildrenIfEmpty()
   {
      getChildren().stream().filter(child -> !hasParameters(child)).forEach(child -> child.setExpanded(true));
   }

   private static boolean hasParameters(TreeItem<ParameterTreeValue> item)
   {
      return !item.getChildren().filtered(child -> !child.getValue().isRegistry()).isEmpty();
   }
}

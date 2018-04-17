package us.ihmc.parameterTuner.guiElements.tree;

import javafx.scene.control.TreeItem;

public class ParameterTreeItem extends TreeItem<ParameterTreeValue>
{
   private static final int minParametersToExpand = 4;

   public ParameterTreeItem(ParameterTreeValue parameterTreeValue)
   {
      super(parameterTreeValue);
      expandedProperty().addListener((observable, oldValue, newValue) -> expandChildrenForSmallRegistries());
   }

   public void expandChildrenForSmallRegistries()
   {
      getChildren().stream().filter(child -> !hasParameters(child, minParametersToExpand)).forEach(child -> child.setExpanded(true));
   }

   private static boolean hasParameters(TreeItem<ParameterTreeValue> item, int minSize)
   {
      return item.getChildren().filtered(child -> !child.getValue().isRegistry()).size() > minSize;
   }
}

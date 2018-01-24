package us.ihmc.robotics.parameterGui.tree;

import java.util.List;

import javafx.scene.control.TreeView;
import us.ihmc.robotics.parameterGui.GuiParameter;
import us.ihmc.robotics.parameterGui.GuiRegistry;
import us.ihmc.robotics.parameterGui.RegularExpression;

public class ParameterTree extends TreeView<ParameterTreeValue>
{
   public ParameterTree()
   {
      super();
      setCellFactory(param -> new ParameterTreeCell());
   }

   public void setRegistries(GuiRegistry registry, boolean hideNamespaces, String regex)
   {
      ParameterTreeItem root = new ParameterTreeItem(null);
      root.setExpanded(true);
      setShowRoot(false);
      setRoot(root);

      boolean searching = regex != null && !regex.isEmpty();
      if (hideNamespaces && searching)
      {
         addAllMatching(registry.getAllParameters(), root, regex);
      }
      else if (hideNamespaces)
      {
         addAll(registry.getAllParameters(), root);
      }
      else if (searching)
      {
         addMatchingRecursive(registry, root, regex);
         sortChildren(root);
      }
      else
      {
         addRecursive(registry, root);
         root.expandChildrenIfEmpty();
      }
   }

   private static void sortChildren(ParameterTreeItem item)
   {
      item.getChildren().sort((o1, o2) -> {
         if (o1.getValue().isRegistry() && !o2.getValue().isRegistry())
         {
            return -1;
         }
         if (o2.getValue().isRegistry() && !o1.getValue().isRegistry())
         {
            return 1;
         }
         return 0;
      });
   }

   private static void addMatchingRecursive(GuiRegistry registry, ParameterTreeItem item, String regex)
   {
      if (RegularExpression.check(registry.getName(), regex))
      {
         addRecursive(registry, item);
      }
      registry.getRegistries().stream().forEach(child -> {
         addMatchingRecursive(child, item, regex);
      });
      addAllMatching(registry.getParameters(), item, regex);
   }

   private static void addRecursive(GuiRegistry registry, ParameterTreeItem item)
   {
      ParameterTreeItem registryItem = new ParameterTreeItem(new ParameterTreeRegistry(registry));
      item.getChildren().add(registryItem);
      registry.getRegistries().stream().forEach(child -> {
         addRecursive(child, registryItem);
      });
      addAll(registry.getParameters(), registryItem);
   }

   private static void addAllMatching(List<GuiParameter> parameters, ParameterTreeItem item, String regex)
   {
      parameters.stream().filter(parameter -> RegularExpression.check(parameter.getName(), regex))
                .forEach(parameter -> item.getChildren().add(new ParameterTreeItem(new ParameterTreeParameter(parameter))));
   }

   private static void addAll(List<GuiParameter> parameters, ParameterTreeItem item)
   {
      parameters.stream().forEach(parameter -> item.getChildren().add(new ParameterTreeItem(new ParameterTreeParameter(parameter))));
   }
}

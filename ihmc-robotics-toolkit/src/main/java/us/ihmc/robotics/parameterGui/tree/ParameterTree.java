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

   public void setRegistries(GuiRegistry registry, boolean hideNamespaces, String regexParameters, String regexNamespaces)
   {
      if (registry == null)
      {
         return;
      }

      ParameterTreeItem root = new ParameterTreeItem(null);
      root.setExpanded(true);
      setShowRoot(false);
      setRoot(root);

      boolean searchingParameters = regexParameters != null && !regexParameters.isEmpty();
      boolean searchingNamespaces = regexNamespaces != null && !regexNamespaces.isEmpty();
      boolean searching = searchingParameters || searchingNamespaces;

      if (hideNamespaces && searching)
      {
         addAllMatching(registry.getAllParameters(), root, regexParameters);
      }
      else if (hideNamespaces)
      {
         addAllMatching(registry.getAllParameters(), root, "");
      }
      else if (searching)
      {
         addMatchingRecursive(registry, root, regexParameters, regexNamespaces);
      }
      else
      {
         addRecursive(registry, root);
         root.expandChildrenIfEmpty();
      }
   }

   private static void addMatchingRecursive(GuiRegistry registry, ParameterTreeItem item, String regexParameters, String regexNamespaces)
   {
      if (RegularExpression.check(registry.getName(), regexNamespaces))
      {
         ParameterTreeItem registryItem = new ParameterTreeItem(new ParameterTreeRegistry(registry));
         registryItem.setExpanded(true);
         addAllMatching(registry.getParameters(), registryItem, regexParameters);
         if (!registryItem.getChildren().isEmpty())
         {
            item.getChildren().add(registryItem);
         }
      }

      registry.getRegistries().stream().forEach(child -> {
         addMatchingRecursive(child, item, regexParameters, regexNamespaces);
      });
   }

   private static void addRecursive(GuiRegistry registry, ParameterTreeItem item)
   {
      ParameterTreeItem registryItem = new ParameterTreeItem(new ParameterTreeRegistry(registry));
      item.getChildren().add(registryItem);
      registry.getRegistries().stream().forEach(child -> {
         addRecursive(child, registryItem);
      });
      addAllMatching(registry.getParameters(), registryItem, "");
   }

   private static void addAllMatching(List<GuiParameter> parameters, ParameterTreeItem item, String regex)
   {
      parameters.stream().filter(parameter -> RegularExpression.check(parameter.getName(), regex))
                .forEach(parameter -> item.getChildren().add(new ParameterTreeItem(new ParameterTreeParameter(parameter))));
   }
}

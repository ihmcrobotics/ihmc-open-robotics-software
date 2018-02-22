package us.ihmc.parameterTuner.guiElements.tree;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.scene.control.TreeView;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiParameterStatus;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.tools.string.RegularExpression;

public class ParameterTree extends TreeView<ParameterTreeValue>
{
   private GuiRegistry registry;
   private final Map<String, ParameterTreeItem> treeItemMap = new HashMap<>();

   public ParameterTree()
   {
      super();
      setCellFactory(param -> new ParameterTreeCell());
   }

   public void setRegistries(GuiRegistry registry)
   {
      this.registry = registry;
      treeItemMap.clear();
      createItemsRecursive(registry);
   }

   private void createItemsRecursive(GuiRegistry registry)
   {
      ParameterTreeItem registryItem = new ParameterTreeItem(new ParameterTreeRegistry(registry));
      treeItemMap.put(registry.getUniqueName(), registryItem);
      registry.getRegistries().stream().forEach(child -> {
         createItemsRecursive(child);
      });
      registry.getParameters().stream().forEach(parameter -> {
         ParameterTreeItem parameterItem = new ParameterTreeItem(new ParameterTreeParameter(parameter));
         treeItemMap.put(parameter.getUniqueName(), parameterItem);
      });
   }

   public void filterRegistries(boolean hideNamespaces, GuiParameterStatus status, String regexParameters, String regexNamespaces)
   {
      if (registry == null)
      {
         return;
      }

      treeItemMap.values().stream().forEach(item -> {
         item.getChildren().clear();
      });

      ParameterTreeItem root = new ParameterTreeItem(null);
      root.setExpanded(true);
      setShowRoot(false);
      setRoot(root);

      boolean searchingParameters = regexParameters != null && !regexParameters.isEmpty();
      boolean searchingNamespaces = regexNamespaces != null && !regexNamespaces.isEmpty();
      boolean searching = searchingParameters || searchingNamespaces || status != GuiParameterStatus.ANY;

      if (hideNamespaces && searching)
      {
         addAllMatching(registry.getAllParameters(), root, regexParameters, status);
      }
      else if (hideNamespaces)
      {
         addAllMatching(registry.getAllParameters(), root, "", status);
      }
      else if (searching)
      {
         addMatchingRecursive(registry, root, regexParameters, regexNamespaces, status);
      }
      else
      {
         addRecursive(registry, root, status);
         root.expandChildrenForSmallRegistries();
      }
   }

   private void addMatchingRecursive(GuiRegistry registry, ParameterTreeItem item, String regexParameters, String regexNamespaces, GuiParameterStatus status)
   {
      if (RegularExpression.check(registry.getName(), regexNamespaces))
      {
         ParameterTreeItem registryItem = treeItemMap.get(registry.getUniqueName());
         registryItem.setExpanded(true);
         addAllMatching(registry.getParameters(), registryItem, regexParameters, status);
         if (!registryItem.getChildren().isEmpty())
         {
            item.getChildren().add(registryItem);
         }
      }

      registry.getRegistries().stream().forEach(child -> {
         addMatchingRecursive(child, item, regexParameters, regexNamespaces, status);
      });
   }

   private void addRecursive(GuiRegistry registry, ParameterTreeItem item, GuiParameterStatus status)
   {
      ParameterTreeItem registryItem = treeItemMap.get(registry.getUniqueName());
      item.getChildren().add(registryItem);
      registry.getRegistries().stream().forEach(child -> {
         addRecursive(child, registryItem, status);
      });
      addAllMatching(registry.getParameters(), registryItem, "", status);
   }

   private void addAllMatching(List<GuiParameter> parameters, ParameterTreeItem item, String regex, GuiParameterStatus status)
   {
      parameters.stream().filter(parameter -> RegularExpression.check(parameter.getName(), regex) && status.matches(parameter)).forEach(parameter -> {
         ParameterTreeItem parameterItem = treeItemMap.get(parameter.getUniqueName());
         item.getChildren().add(parameterItem);
      });
   }
}

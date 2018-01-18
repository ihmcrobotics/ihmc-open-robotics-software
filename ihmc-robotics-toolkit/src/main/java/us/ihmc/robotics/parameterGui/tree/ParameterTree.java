package us.ihmc.robotics.parameterGui.tree;

import java.util.Comparator;
import java.util.List;

import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import us.ihmc.robotics.parameterGui.RegularExpression;
import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterTree extends TreeView<ParameterTreeValue>
{
   public ParameterTree()
   {
      super();
      setCellFactory(param -> new ParameterTreeCell());
   }

   public void setRegistries(List<Registry> registries, boolean hideNamespaces, String regex)
   {
      TreeItem<ParameterTreeValue> root = new TreeItem<>();
      root.setExpanded(true);
      setShowRoot(false);
      setRoot(root);
      registries.stream().forEach(registry -> recursiveAddRegistry(registry, root, hideNamespaces, regex));

      root.getChildren().sort(new Comparator<TreeItem<ParameterTreeValue>>()
      {
         @Override
         public int compare(TreeItem<ParameterTreeValue> o1, TreeItem<ParameterTreeValue> o2)
         {
            if (o1.getValue().isRegistry() && !o2.getValue().isRegistry())
            {
               return -1;
            }
            if (o2.getValue().isRegistry() && !o1.getValue().isRegistry())
            {
               return 1;
            }
            return 0;
         }
      });
   }

   private static void recursiveAddRegistry(Registry registry, TreeItem<ParameterTreeValue> root, boolean hideNamespaces, String regex)
   {
      if (registry == null)
      {
         return;
      }

      boolean searching = regex != null && !regex.isEmpty();
      List<Parameter> parameters = registry.getParameters();
      List<Registry> children = registry.getRegistries();

      if (children != null)
      {
         for (Registry child : children)
         {
            ParameterTreeValue childValue = new ParameterTreeRegistry(child);
            TreeItem<ParameterTreeValue> childItem = new TreeItem<>(childValue);
            boolean noParameters = child.getParameters() == null || child.getParameters().isEmpty();
            if (!searching && noParameters)
            {
               childItem.setExpanded(true);
            }

            if (!searching)
            {
               if (hideNamespaces)
               {
                  recursiveAddRegistry(child, root, hideNamespaces, regex);
               }
               else
               {
                  root.getChildren().add(childItem);
                  recursiveAddRegistry(child, childItem, hideNamespaces, regex);
               }
            }
            else
            {
               if (!hideNamespaces && RegularExpression.check(child.getName(), regex))
               {
                  root.getChildren().add(childItem);
                  recursiveAddRegistry(child, childItem, hideNamespaces, null);
               }
               recursiveAddRegistry(child, root, hideNamespaces, regex);
            }

         }
      }

      if (parameters != null)
      {
         for (Parameter parameter : parameters)
         {
            ParameterTreeValue parameterValue = new ParameterTreeParameter(parameter);
            TreeItem<ParameterTreeValue> parameterItem = new TreeItem<>(parameterValue);

            if (!searching)
            {
               root.getChildren().add(parameterItem);
            }
            else
            {
               if (RegularExpression.check(parameter.getName(), regex))
               {
                  root.getChildren().add(parameterItem);
               }
            }
         }
      }
   }
}

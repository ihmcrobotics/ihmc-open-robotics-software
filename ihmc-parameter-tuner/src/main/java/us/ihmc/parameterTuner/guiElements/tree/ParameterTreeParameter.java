package us.ihmc.parameterTuner.guiElements.tree;

import org.apache.commons.lang3.StringUtils;

import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SeparatorMenuItem;
import javafx.scene.control.Tooltip;
import javafx.scene.control.TreeItem;
import javafx.scene.input.Clipboard;
import javafx.scene.input.ClipboardContent;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiElement;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiParameterStatus;

public class ParameterTreeParameter implements ParameterTreeValue
{
   private final GuiParameter parameter;
   private final Node treeTuner;
   private final ObservableList<TreeItem<ParameterTreeValue>> selectedItems;

   private ParameterNode node;

   public ParameterTreeParameter(GuiParameter parameter, Node treeTuner, ObservableList<TreeItem<ParameterTreeValue>> selectedItems)
   {
      this.parameter = parameter;
      this.treeTuner = treeTuner;
      this.selectedItems = selectedItems;
   }

   @Override
   public boolean isRegistry()
   {
      return false;
   }

   @Override
   public String getName()
   {
      return parameter.getName();
   }

   @Override
   public GuiParameter getParameter()
   {
      return parameter;
   }

   @Override
   public Node getOrCreateNode()
   {
      if (node == null)
      {
         node = new ParameterNode(parameter, treeTuner);
      }
      return node;
   }

   private class ParameterNode extends HBox
   {
      private final Text name = new Text();

      private final ContextMenu contextMenu = new ContextMenu();
      private final MenuItem discard = new MenuItem("Discard");
      private final MenuItem markModified = new MenuItem("Mark as Modified");
      private final MenuItem copyName = new MenuItem("Copy Name");
      private final MenuItem copyNamespace = new MenuItem("Copy Namespace");
      private final Clipboard clipboard = Clipboard.getSystemClipboard();
      private final ClipboardContent content = new ClipboardContent();

      public ParameterNode(GuiParameter parameter, Node treeTuner)
      {
         // Creating this using fxml causes crazy slow down since it is done a lot.
         setSpacing(10.0);
         setAlignment(Pos.CENTER_LEFT);
         getChildren().add(treeTuner);
         getChildren().add(name);
         setOnContextMenuRequested((event) -> contextMenu.show(name, event.getScreenX(), event.getScreenY()));

         // Setup context menu for copying name to system clipboard.
         contextMenu.getItems().add(copyName);
         copyName.setOnAction((event) -> {
            content.putString(parameter.getName());
            clipboard.setContent(content);
         });
         contextMenu.getItems().add(copyNamespace);
         copyNamespace.setOnAction((event) -> {
            String[] splitNamespace = StringUtils.split(parameter.getUniqueName(), GuiElement.SEPERATOR);
            content.putString(splitNamespace[splitNamespace.length - 2]);
            clipboard.setContent(content);
         });
         contextMenu.getItems().add(new SeparatorMenuItem());

         // Setup context menu for discarding changes.
         contextMenu.getItems().add(discard);
         discard.setOnAction(event -> {
            selectedItems.filtered(item -> !item.getValue().isRegistry()).forEach(item -> item.getValue().getParameter().reset());
         });

         // Setup context menu for marking a parameter as modified.
         contextMenu.getItems().add(markModified);
         markModified.setOnAction(event -> {
            selectedItems.filtered(item -> !item.getValue().isRegistry()).forEach(item -> item.getValue().getParameter().markAsModified());
         });

         // Set up the css styles for the parameter status.
         updateStyle(parameter);

         parameter.addChangedListener(p -> Platform.runLater(() -> {
            updateStyle(parameter);
         }));

         name.setText(parameter.getName());

         Tooltip tooltip = new Tooltip(StringUtils.replaceAll(parameter.getUniqueName(), GuiElement.SEPERATOR, "\n"));
         Tooltip.install(this, tooltip);
      }

      private void updateStyle(GuiParameter parameter)
      {
         if (parameter.getStatus() == GuiParameterStatus.DEFAULT)
         {
            name.setId("default-parameter-name-in-tree-view");
         }
         else if (parameter.getStatus() == GuiParameterStatus.MODIFIED)
         {
            name.setId("modified-parameter-name-in-tree-view");
         }
         else
         {
            name.setId("parameter-name-in-tree-view");
         }
      }
   }
}

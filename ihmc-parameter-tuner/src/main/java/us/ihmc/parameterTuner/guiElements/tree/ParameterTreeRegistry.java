package us.ihmc.parameterTuner.guiElements.tree;

import org.apache.commons.lang3.StringUtils;

import javafx.application.Platform;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tooltip;
import javafx.scene.input.Clipboard;
import javafx.scene.input.ClipboardContent;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiElement;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;

public class ParameterTreeRegistry implements ParameterTreeValue
{
   private final GuiRegistry registry;
   private Node node;

   public ParameterTreeRegistry(GuiRegistry registry)
   {
      this.registry = registry;
   }

   @Override
   public boolean isRegistry()
   {
      return true;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public Node getOrCreateNode()
   {
      if (node == null)
      {
         node = new RegistryNode(registry);
      }
      return node;
   }

   @Override
   public GuiParameter getParameter()
   {
      throw new RuntimeException("Is a registry!");
   }

   private class RegistryNode extends HBox
   {
      private final Text name = new Text();

      private final ContextMenu contextMenu = new ContextMenu();
      private final MenuItem makeRoot = new MenuItem("Make Root");
      private final MenuItem copyName = new MenuItem("Copy Name");
      private final Clipboard clipboard = Clipboard.getSystemClipboard();
      private final ClipboardContent content = new ClipboardContent();

      public RegistryNode(GuiRegistry registry)
      {
         // Creating this using fxml causes crazy slow down since it is done a lot.
         setSpacing(10.0);
         setAlignment(Pos.CENTER_LEFT);
         getChildren().add(name);
         setOnContextMenuRequested((event) -> contextMenu.show(name, event.getScreenX(), event.getScreenY()));

         // Setup context menu for copying name to system clipboard.
         contextMenu.getItems().add(copyName);
         copyName.setOnAction((event) -> {
            content.putString(registry.getName());
            clipboard.setContent(content);
         });

         // Setup context menu for making this a root registry.
         contextMenu.getItems().add(makeRoot);
         makeRoot.setOnAction(event -> Platform.runLater(() -> registry.makeRoot()));

         // Set up the css styles for the parameter status.
         registry.isRoot().addListener((observable, oldValue, newValue) -> updateStyle(newValue));
         updateStyle(registry.isRoot().get());

         name.setText(registry.getName());

         Tooltip tooltip = new Tooltip(StringUtils.replaceAll(registry.getUniqueName(), GuiElement.SEPERATOR, "\n"));
         Tooltip.install(this, tooltip);
      }

      private void updateStyle(boolean isRoot)
      {
         if (isRoot)
         {
            name.setId("root-registry-name-in-tree-view");
         }
         else
         {
            name.setId("registry-name-in-tree-view");
         }
      }
   }
}

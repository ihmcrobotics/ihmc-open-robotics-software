package us.ihmc.parameterTuner.guiElements.tree;

import javafx.application.Platform;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.Label;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiParameterStatus;

public class ParameterTreeParameter implements ParameterTreeValue
{
   private final GuiParameter parameter;
   private ParameterNode node;

   public ParameterTreeParameter(GuiParameter parameter)
   {
      this.parameter = parameter;
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

   public GuiParameter getParameter()
   {
      return parameter;
   }

   @Override
   public Node getOrCreateNode()
   {
      if (node == null)
      {
         node = new ParameterNode(parameter);
      }
      return node;
   }

   private class ParameterNode extends HBox
   {
      private final Text name = new Text();
      private final Label value = new Label();
      private final MenuItem discard = new MenuItem("Discard");

      public ParameterNode(GuiParameter parameter)
      {
         // Creating this using fxml causes crazy slow down since it is done a lot.
         value.setPrefWidth(80.0);
         setSpacing(10.0);
         setAlignment(Pos.CENTER_LEFT);
         getChildren().add(value);
         getChildren().add(name);
         value.setId("parameter-value-in-tree-view");

         // Setup context menu for discarding changes.
         ContextMenu contextMenu = new ContextMenu();
         discard.setDisable(true);
         contextMenu.getItems().add(discard);
         setOnContextMenuRequested((event) -> contextMenu.show(value, event.getScreenX(), event.getScreenY()));
         discard.setOnAction(event -> {
            parameter.reset();
         });

         // Set up the css styles for the parameter status.
         updateStyle(parameter);

         parameter.addChangedListener(p -> Platform.runLater(() -> {
            updateStyle(parameter);
            value.setText(parameter.getCurrentValue());
         }));

         name.setText(parameter.getName());
         value.setText(parameter.getCurrentValue());

         Tooltip tooltip = new Tooltip();
         tooltip.setText(parameter.getCurrentDescription());
         parameter.addChangedListener(p -> tooltip.setText(parameter.getCurrentDescription()));
         Tooltip.install(this, tooltip);
      }

      private void updateStyle(GuiParameter parameter)
      {
         if (parameter.getStatus() == GuiParameterStatus.DEFAULT)
         {
            discard.setDisable(true);
            name.setId("default-parameter-name-in-tree-view");
         }
         else if (parameter.getStatus() == GuiParameterStatus.MODIFIED)
         {
            discard.setDisable(false);
            name.setId("modified-parameter-name-in-tree-view");
         }
         else
         {
            discard.setDisable(true);
            name.setId("parameter-name-in-tree-view");
         }
      }
   }
}

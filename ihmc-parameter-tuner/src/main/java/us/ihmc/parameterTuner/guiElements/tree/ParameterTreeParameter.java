package us.ihmc.parameterTuner.guiElements.tree;

import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

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

      public ParameterNode(GuiParameter parameter)
      {
         // Creating this using fxml causes crazy slow down since it is done a lot.
         value.setPrefWidth(80.0);
         setSpacing(10.0);
         setAlignment(Pos.CENTER_LEFT);
         getChildren().add(value);
         getChildren().add(name);
         value.setId("parameter-value-in-tree-view");
         name.setId("parameter-name-in-tree-view");

         parameter.addChangedListener(p -> value.setText(parameter.getCurrentValue()));
         name.setText(parameter.getName());
         value.setText(parameter.getCurrentValue());

         Tooltip tooltip = new Tooltip();
         tooltip.setText(parameter.getCurrentDescription());
         parameter.addChangedListener(p -> tooltip.setText(parameter.getCurrentDescription()));
         Tooltip.install(this, tooltip);
      }
   }
}

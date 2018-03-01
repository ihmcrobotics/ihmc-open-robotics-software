package us.ihmc.parameterTuner.guiElements.tuners;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class BooleanTuner extends HBox implements InputNode
{
   private final CheckBox value = new CheckBox();

   public BooleanTuner(GuiParameter parameter)
   {
      setupNode();

      value.setSelected(Boolean.parseBoolean(parameter.getCurrentValue()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         value.setSelected(Boolean.parseBoolean(parameter.getCurrentValue()));
      });

      value.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> {
         Platform.runLater(() -> parameter.setValue(Boolean.toString(value.isSelected())));
      });
   }

   private void setupNode()
   {
      setSpacing(10.0);
      setAlignment(Pos.CENTER_LEFT);
      setMaxHeight(Double.NEGATIVE_INFINITY);
      setMaxWidth(Double.NEGATIVE_INFINITY);
      setPrefWidth(800.0);
      setPadding(new Insets(0.0, 5.0, 5.0, 5.0));
      getChildren().add(new Text("Value"));
      getChildren().add(value);
   }


   @Override
   public Node getSimpleInputNode(double width, double height)
   {
      CheckBox duplicate = new CheckBox(value.getText());
      duplicate.setPrefHeight(height);
      duplicate.setPrefWidth(width);
      duplicate.setSelected(value.isSelected());
      duplicate.selectedProperty().addListener((observable, oldValue, newValue) -> value.setSelected(newValue));
      value.selectedProperty().addListener((observable, oldValue, newValue) -> duplicate.setSelected(newValue));
      return duplicate;
   }
}

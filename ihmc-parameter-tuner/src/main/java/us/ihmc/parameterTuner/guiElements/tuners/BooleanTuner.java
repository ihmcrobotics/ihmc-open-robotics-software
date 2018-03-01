package us.ihmc.parameterTuner.guiElements.tuners;

import java.io.IOException;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.HBox;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class BooleanTuner extends HBox implements InputNode
{
   private static final String FXML_PATH = "boolean_tuner.fxml";

   @FXML
   private CheckBox value;

   public BooleanTuner(GuiParameter parameter)
   {
      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      try
      {
         loader.load();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      value.setSelected(Boolean.parseBoolean(parameter.getCurrentValue()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         value.setSelected(Boolean.parseBoolean(parameter.getCurrentValue()));
      });

      value.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> {
         Platform.runLater(() -> parameter.setValue(Boolean.toString(value.isSelected())));
      });
   }

   @Override
   public Node getSimpleInputNode()
   {
      CheckBox duplicate = new CheckBox(value.getText());
      duplicate.setSelected(value.isSelected());
      duplicate.selectedProperty().addListener((observable, oldValue, newValue) -> value.setSelected(newValue));
      value.selectedProperty().addListener((observable, oldValue, newValue) -> duplicate.setSelected(newValue));
      return duplicate;
   }
}

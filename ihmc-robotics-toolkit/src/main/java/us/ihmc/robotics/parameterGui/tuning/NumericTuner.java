package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.HBox;
import us.ihmc.yoVariables.parameters.xml.Parameter;

public class NumericTuner extends HBox
{
   private static final String FXML_PATH = "numeric_tuner.fxml";

   @FXML
   private NumericTextField value;

   @FXML
   private NumericTextField min;

   @FXML
   private NumericTextField max;

   public NumericTuner(Parameter parameter)
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

      double initialValue = Double.parseDouble(parameter.getValue());
      double initialMin = Double.parseDouble(parameter.getMin());
      double initialMax = Double.parseDouble(parameter.getMax());

      if (initialValue < initialMin || initialValue > initialMax)
      {
         Alert alert = new Alert(AlertType.INFORMATION);
         alert.setTitle("Information Dialog");
         alert.setHeaderText("Bound Inconsistency");
         alert.setContentText("Setting the bounds such that value is valid.");
         alert.showAndWait();

         initialMin = Math.min(initialMin, initialValue);
         initialMax = Math.max(initialMax, initialValue);
      }

      value.addListener((observable, oldValue, newValue) -> parameter.setValue(value.getText()));
      min.addListener((observable, oldValue, newValue) -> {
         value.setMinValue(min.getValue());
         max.setMinValue(min.getValue());
         parameter.setMin(min.getText());
      });
      max.addListener((observable, oldValue, newValue) -> {
         value.setMaxValue(max.getValue());
         min.setMaxValue(max.getValue());
         parameter.setMax(max.getText());
      });

      value.set(initialValue);
      min.set(initialMin);
      max.set(initialMax);
   }
}

package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.parameterGui.GuiParameter;

public abstract class NumericTuner <T extends Number> extends HBox
{
   private static final String FXML_PATH = "numeric_tuner.fxml";

   @FXML
   private StackPane valuePane;
   private final NumericSpinner<T> value = createASpinner();

   @FXML
   private StackPane minPane;
   private final NumericSpinner<T> min = createASpinner();

   @FXML
   private StackPane maxPane;
   private final NumericSpinner<T> max = createASpinner();

   public NumericTuner(GuiParameter parameter)
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

      valuePane.getChildren().add(value);
      minPane.getChildren().add(min);
      maxPane.getChildren().add(max);

      NumericSpinner<T> spinner = createASpinner();
      T initialValue = spinner.convertStringToNumber(parameter.getCurrentValue());
      T initialMin = spinner.convertStringToNumber(parameter.getCurrentMin());
      T initialMax = spinner.convertStringToNumber(parameter.getCurrentMax());

      if (!areBoundsConsistent(initialValue, initialMin, initialMax))
      {
         Alert alert = new Alert(AlertType.INFORMATION);
         alert.setTitle("Information Dialog");
         alert.setHeaderText("Bound Inconsistency");
         alert.setContentText("Setting the bounds such that value is valid.");
         alert.showAndWait();

         initialMin = getSmallerNumber(initialMin, initialValue);
         initialMax = getLargerNumber(initialMax, initialValue);
      }

      value.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> {
            parameter.setValue(value.getValueAsText());
         });
      });

      min.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> {
            max.setMinValue(min.getValue());
            value.setMinValue(min.getValue());
            parameter.setMin(min.getValueAsText());
         });
      });

      max.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> {
            min.setMaxValue(max.getValue());
            value.setMaxValue(max.getValue());
            parameter.setMax(max.getValueAsText());
         });
      });

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         T newValue = value.convertStringToNumber(parameter.getCurrentValue());
         T newMin = min.convertStringToNumber(parameter.getCurrentMin());
         T newMax = max.convertStringToNumber(parameter.getCurrentMax());

         if (!areBoundsConsistent(newValue, newMin, newMax))
         {
            PrintTools.warn(parameter.getName() + ": Parameter bound was changed externally to be inconsistent.");
            T adjustedMin = getSmallerNumber(newMin, newValue);
            T adjustedMax = getLargerNumber(newMax, newValue);
            Platform.runLater(() -> set(newValue, adjustedMin, adjustedMax));
         }
         else
         {
            set(newValue, newMin, newMax);
         }
      });

      set(initialValue, initialMin, initialMax);
   }

   private void set(T newValue, T newMin, T newMax)
   {
      value.setValue(newValue);
      min.setValue(newMin);
      max.setValue(newMax);
   }

   public abstract NumericSpinner<T> createASpinner();

   public abstract boolean areBoundsConsistent(T value, T min, T max);

   public abstract T getSmallerNumber(T a, T b);

   public abstract T getLargerNumber(T a, T b);
}

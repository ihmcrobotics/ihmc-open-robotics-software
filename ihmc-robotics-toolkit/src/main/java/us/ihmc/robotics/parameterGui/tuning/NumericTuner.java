package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
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

      value.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> {
            parameter.setValue(value.getValueAsText());
         });
      });

      min.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> {
            max.setMinValue(min.getValue());
            parameter.setMin(min.getValueAsText());
         });
      });

      max.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> {
            min.setMaxValue(max.getValue());
            parameter.setMax(max.getValueAsText());
         });
      });

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         setFromParameter(parameter);
      });
      setFromParameter(parameter);
   }

   private void setFromParameter(GuiParameter parameter)
   {
      setFromString(parameter.getCurrentValue(), value);
      setFromString(parameter.getCurrentMin(), min);
      setFromString(parameter.getCurrentMax(), max);
   }

   private static <T extends Number> void setFromString(String numberString, NumericSpinner<T> numberToPack)
   {
      T newValue = numberToPack.convertStringToNumber(numberString);
      numberToPack.setValue(newValue);
   }

   public abstract NumericSpinner<T> createASpinner();

   public abstract T getSmallerNumber(T a, T b);

   public abstract T getLargerNumber(T a, T b);
}

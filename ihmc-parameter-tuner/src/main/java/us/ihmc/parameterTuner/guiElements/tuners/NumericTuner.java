package us.ihmc.parameterTuner.guiElements.tuners;

import java.io.IOException;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public abstract class NumericTuner<T extends Number> extends HBox implements InputNode
{
   private static final String FXML_PATH = "numeric_tuner.fxml";

   @FXML
   private StackPane valuePane;
   private final NumericSpinner<T> value = createSpinner();

   @FXML
   private StackPane minPane;
   private final NumericSpinner<T> min = createSpinner();

   @FXML
   private StackPane maxPane;
   private final NumericSpinner<T> max = createSpinner();

   @FXML
   private StackPane sliderPane;
   private NumericSlider<T> slider = createSlider();

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
      sliderPane.getChildren().add(slider);

      value.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> parameter.setValue(value.getValueAsText()));
      });

      min.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> parameter.setMin(min.getValueAsText()));
      });

      max.addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> parameter.setMax(max.getValueAsText()));
      });

      // Use the slider value when it gets clicked or dragged and released.
      slider.valueChangingProperty().addListener((observable, oldValue, newValue) -> {
         if (!newValue && oldValue)
         {
            value.setValue(slider.getNumber());
         }
      });
      slider.setOnMouseClicked(mouseEvent -> value.setValue(slider.getNumber()));

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
      slider.updateSlider(value.getValue(), min.getValue(), max.getValue());
   }

   private static <T extends Number> void setFromString(String numberString, NumericSpinner<T> numberToPack)
   {
      T newValue = numberToPack.convertStringToNumber(numberString);
      numberToPack.setValue(newValue);
   }

   public abstract NumericSpinner<T> createSpinner();

   public abstract NumericSlider<T> createSlider();

   @Override
   public Node getSimpleInputNode()
   {
      return value.createLinkedDuplicate();
   }
}

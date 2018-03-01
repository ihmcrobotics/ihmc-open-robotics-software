package us.ihmc.parameterTuner.guiElements.tuners;

import javafx.application.Platform;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public abstract class NumericTuner<T extends Number> extends HBox implements InputNode
{
   private final NumericSpinner<T> value = createSpinner();
   private final NumericSpinner<T> min = createSpinner();
   private final NumericSpinner<T> max = createSpinner();
   private final NumericSlider<T> slider = createSlider();

   public NumericTuner(GuiParameter parameter)
   {
      setupNode();

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
      getChildren().add(new Text("Min"));
      getChildren().add(min);
      getChildren().add(slider);
      getChildren().add(new Text("Max"));
      getChildren().add(max);
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

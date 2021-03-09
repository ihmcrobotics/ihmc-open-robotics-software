package us.ihmc.javafx.parameter;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.control.Slider;

public class JavaFXDoubleSliderPropertyHolder extends JavaFXPropertyHolder<Double>
{
   private final Slider slider;

   private final ChangeListener doubleSliderChanged = this::doubleSliderChanged;

   public JavaFXDoubleSliderPropertyHolder(Slider slider)
   {
      this.slider = slider;

      slider.valueProperty().addListener(doubleSliderChanged);
      slider.valueChangingProperty().addListener(changingListener);
   }

   public void doubleSliderChanged(ObservableValue observable, Object oldValue, Object newValue)
   {
      changeListener.changed(observable, (Double) oldValue, (Double) newValue);
   }

   @Override
   public Double getValue()
   {
      return slider.getValue();
   }

   @Override
   protected void setValueInternal(Double value)
   {
      slider.valueProperty().setValue(value);
   }
}

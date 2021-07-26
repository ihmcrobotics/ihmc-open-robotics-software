package us.ihmc.javafx.parameter;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.control.Slider;

public class JavaFXIntegerSliderPropertyHolder extends JavaFXPropertyHolder<Integer>
{
   private final Slider slider;

   private final ChangeListener integerSliderChanged = this::integerSliderChanged;

   public JavaFXIntegerSliderPropertyHolder(Slider slider)
   {
      this.slider = slider;

      slider.valueProperty().addListener(integerSliderChanged);
      slider.valueChangingProperty().addListener(changingListener);
   }

   public void integerSliderChanged(ObservableValue observable, Object oldValue, Object newValue)
   {
      changeListener.changed(observable, (Integer) oldValue, (Integer) newValue);
   }

   @Override
   public Integer getValue()
   {
      return (int) slider.getValue();
   }

   @Override
   protected void setValueInternal(Integer value)
   {
      slider.valueProperty().setValue(value);
   }
}

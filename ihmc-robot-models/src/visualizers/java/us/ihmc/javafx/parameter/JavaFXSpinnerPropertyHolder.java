package us.ihmc.javafx.parameter;

import javafx.scene.control.Spinner;

public class JavaFXSpinnerPropertyHolder<T> extends JavaFXPropertyHolder<T>
{
   private final Spinner<T> spinner;

   public JavaFXSpinnerPropertyHolder(Spinner<T> spinner)
   {
      this.spinner = spinner;

      spinner.getValueFactory().valueProperty().addListener(changeListener);
   }

   @Override
   public T getValue()
   {
      return spinner.getValueFactory().valueProperty().getValue();
   }

   @Override
   protected void setValueInternal(T value)
   {
      spinner.getValueFactory().valueProperty().setValue(value);
   }
}

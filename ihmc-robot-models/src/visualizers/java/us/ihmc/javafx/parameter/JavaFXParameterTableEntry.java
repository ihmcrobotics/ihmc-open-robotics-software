package us.ihmc.javafx.parameter;

import javafx.beans.InvalidationListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import us.ihmc.javafx.FunctionalObservableValue;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class JavaFXParameterTableEntry<T>
{
   private final String parameterName;
   private final Supplier<T> getter;
   private final Consumer<T> setter;
   private final ObservableValue<String> observableName;
   private final ObservableValue<Spinner<T>> observableValue;
   private final Spinner<T> spinner;

   public JavaFXParameterTableEntry(String parameterName,
                                    Supplier<T> getter,
                                    Consumer<T> setter,
                                    InvalidationListener onUserSpunSpinner,
                                    SpinnerValueFactory<T> spinnerValueFactory)
   {
      this.parameterName = parameterName;
      this.getter = getter;
      this.setter = setter;
      observableName = new FunctionalObservableValue<>(this::getParameterName);
      observableValue = new FunctionalObservableValue<>(this::getSpinner);

      spinner = new Spinner<>();
      spinner.setValueFactory(spinnerValueFactory);
      spinner.getValueFactory().valueProperty().addListener((observable, old, newValue) -> setter.accept(newValue));
//      spinner.getValueFactory().valueProperty().addListener(observable -> setter.accept(spinner.getValue())); // TODO could also use if above doesn't work
      spinner.getValueFactory().valueProperty().addListener(onUserSpunSpinner);
   }

   public void setValue(T value)
   {
      setter.accept(value);
   }

   public T getValue()
   {
      return getter.get();
   }

   public ObservableValue<String> getObservableName()
   {
      return observableName;
   }

   public ObservableValue<Spinner<T>> getObservableValue()
   {
      return observableValue;
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public Spinner<T> getSpinner()
   {
      return spinner;
   }
}

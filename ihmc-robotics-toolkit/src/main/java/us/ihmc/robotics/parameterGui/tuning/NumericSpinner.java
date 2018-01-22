package us.ihmc.robotics.parameterGui.tuning;

import java.util.function.UnaryOperator;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;
import javafx.scene.input.KeyCode;
import javafx.util.StringConverter;

public abstract class NumericSpinner <T extends Number> extends Spinner<T>
{
   public NumericSpinner(SpinnerValueFactory<T> valueFactory)
   {
      super(valueFactory);

      setEditable(true);

      // Add formatter that will prevent any input that is not a number with a few exceptions (e.g. '-').
      UnaryOperator<TextFormatter.Change> filter = new UnaryOperator<TextFormatter.Change>()
      {
         @Override
         public Change apply(Change change)
         {
            String newText = change.getControlNewText();
            if (newText.isEmpty() || newText.equals("-"))
            {
               return change;
            }
            if (isValidString(newText))
            {
               return change;
            }
            else
            {
               return null;
            }
         }
      };
      getEditor().setTextFormatter(new TextFormatter<>(filter));

      // Add a custom converter that will allow inputing allowed text (e.g. '-') without exception.
      getValueFactory().setConverter(new StringConverter<T>()
      {
         @Override
         public String toString(T number)
         {
            return convertNumberToString(number);
         }

         @Override
         public T fromString(String string)
         {
            try
            {
               return convertStringToNumber(string);
            }
            catch (Exception e)
            {
               return getValue();
            }
         }
      });

      // Update value when enter is pressed or focus is lost.
      getEditor().setOnKeyPressed(event -> {
         if (event.getCode() == KeyCode.ENTER)
         {
            T newNumber = getValueFactory().getConverter().fromString(getEditor().getText());
            Platform.runLater(() -> setValue(newNumber));
         }
      });
      getEditor().focusedProperty().addListener((observable, oldValue, newValue) -> {
         if (oldValue && !newValue)
         {
            T newNumber = getValueFactory().getConverter().fromString(getEditor().getText());
            Platform.runLater(() -> setValue(newNumber));
         }
      });
   }

   public void setValue(T newValue)
   {
      getValueFactory().setValue(newValue);
      getEditor().setText(getValueFactory().getConverter().toString(getValue()));
   }

   public String getText()
   {
      return getEditor().getText();
   }

   public void addListener(ChangeListener<T> listener)
   {
      valueProperty().addListener(listener);
   }

   private boolean isValidString(String numberString)
   {
      try
      {
         convertStringToNumber(numberString);
         return true;
      }
      catch (Exception e)
      {
         return false;
      }
   }

   protected void revalidate()
   {
      setValue(getValue());
   }

   public abstract void setMaxValue(T maxValue);

   public abstract void setMinValue(T minValue);

   public abstract T convertStringToNumber(String numberString);

   public abstract String convertNumberToString(T number);
}

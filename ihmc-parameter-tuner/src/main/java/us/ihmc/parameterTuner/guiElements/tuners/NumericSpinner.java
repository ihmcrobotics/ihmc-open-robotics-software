package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.List;
import java.util.function.UnaryOperator;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;
import javafx.util.StringConverter;
import us.ihmc.parameterTuner.ParameterTuningTools;

public abstract class NumericSpinner<T extends Number> extends Spinner<T>
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
               // So the user can delete all text or start with a minus.
               return change;
            }
            if (isValidString(newText))
            {
               // If the text parses to a number it id fine.
               return change;
            }
            if (change.isDeleted())
            {
               // In case the textbox contained a special string such as "Infinity" the delete action should clear the editor.
               Platform.runLater(() -> getEditor().setText(""));
            }
            return null;
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
               // If the text does not parse (e.g. it is empty or just a minus sign) remember the last value.
               return getValue();
            }
         }
      });

      // Update value when enter is pressed or focus is lost.
      ParameterTuningTools.addThreadSafeListeners(getEditor(), () -> {
         T newNumber = getValueFactory().getConverter().fromString(getEditor().getText());
         setValue(newNumber);
      });

      // Add options for strings that are not allowed to type such as "Infinity" for Double.
      List<MenuItem> contextMenuOptions = getContextMenuOptions();
      if (contextMenuOptions != null)
      {
         ContextMenu contextMenu = new ContextMenu();
         for (MenuItem menuItem : contextMenuOptions)
         {
            contextMenu.getItems().add(menuItem);
         }
         getEditor().setContextMenu(contextMenu);
      }
   }

   public void setValue(T newValue)
   {
      getValueFactory().setValue(newValue);
      getEditor().setText(getValueFactory().getConverter().toString(getValue()));
   }

   public String getValueAsText()
   {
      return convertNumberToString(getValue());
   }

   public void addListener(ChangeListener<T> listener)
   {
      valueProperty().addListener(listener);
   }

   public boolean isValidString(String numberString)
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

   public abstract T convertStringToNumber(String numberString);

   public abstract String convertNumberToString(T number);

   public List<MenuItem> getContextMenuOptions()
   {
      return null;
   }

   public NumericSpinner<T> createLinkedDuplicate()
   {
      return new NumericSpinner<T>(getValueFactory())
      {
         @Override
         public List<MenuItem> getContextMenuOptions()
         {
            return NumericSpinner.this.getContextMenuOptions();
         }

         @Override
         public T convertStringToNumber(String numberString)
         {
            return NumericSpinner.this.convertStringToNumber(numberString);
         }

         @Override
         public String convertNumberToString(T number)
         {
            return NumericSpinner.this.convertNumberToString(number);
         }
      };
   }
}

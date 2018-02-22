package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.List;
import java.util.function.UnaryOperator;

import org.apache.commons.lang3.tuple.ImmutablePair;

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
      ContextMenu contextMenu = new ContextMenu();
      List<ImmutablePair<String, String>> specialStringOptions = getSpecialStringOptions();
      for (ImmutablePair<String, String> option : specialStringOptions)
      {
         MenuItem menuItem = new MenuItem(option.getLeft());
         T number = getValueFactory().getConverter().fromString(option.getRight());
         menuItem.setOnAction(actionEvent -> setValue(number));
         contextMenu.getItems().add(menuItem);
      }
      getEditor().setContextMenu(contextMenu);
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

   public abstract T convertStringToNumber(String numberString);

   public abstract String convertNumberToString(T number);

   public abstract List<ImmutablePair<String, String>> getSpecialStringOptions();
}

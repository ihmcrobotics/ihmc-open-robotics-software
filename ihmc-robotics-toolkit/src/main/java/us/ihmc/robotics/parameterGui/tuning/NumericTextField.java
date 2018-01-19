package us.ihmc.robotics.parameterGui.tuning;

import java.text.DecimalFormat;
import java.text.ParsePosition;
import java.util.function.UnaryOperator;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;
import javafx.scene.input.KeyCode;
import us.ihmc.commons.MathTools;

public class NumericTextField extends TextField
{
   private static final DecimalFormat format = new DecimalFormat("#.0;-#.0");

   private static final String[] allowedStrings = {
         Double.toString(Double.NEGATIVE_INFINITY),
         Double.toString(Double.POSITIVE_INFINITY)
   };

   private double minValue = Double.NEGATIVE_INFINITY;
   private double maxValue = Double.POSITIVE_INFINITY;

   private final DoubleProperty value = new SimpleDoubleProperty(0.0);

   public NumericTextField()
   {
      // Add formatter that will prevent any input that is not a double.
      UnaryOperator<TextFormatter.Change> filter = new UnaryOperator<TextFormatter.Change>()
      {
         @Override
         public Change apply(Change change)
         {
            String newText = change.getControlNewText();
            for (String string : allowedStrings)
            {
               if (string.equals(newText))
               {
                  return change;
               }
            }

            if (newText.isEmpty() || newText.equals("-"))
            {
               return change;
            }

            ParsePosition parsePosition = new ParsePosition(0);
            Object object = format.parse(newText, parsePosition);
            if (object == null || parsePosition.getIndex() < newText.length())
            {
               return null;
            }

            return change;
         }
      };
      setTextFormatter(new TextFormatter<>(filter));

      // Update applied value only when enter is pressed or focus is lost.
      setOnKeyPressed(event ->
      {
         if (event.getCode() == KeyCode.ENTER)
         {
            validateValue();
         }
      });
      focusedProperty().addListener((observable, oldValue, newValue) ->
      {
         if (oldValue && !newValue)
         {
            validateValue();
         }
      });

      // Add context menu entries for special numbers since they can not be typed.
      ContextMenu contextMenu = new ContextMenu();
      for (String string : allowedStrings)
      {
         MenuItem specialStringItem = new MenuItem(string);
         specialStringItem.setOnAction(e -> setText(string));
         contextMenu.getItems().add(specialStringItem);
      }
      setContextMenu(contextMenu);

      setText(Double.toString(0.0));
   }

   private void validateValue()
   {
      try
      {
         double parsed = Double.parseDouble(getText());
         double newValue = MathTools.clamp(parsed, minValue, maxValue);
         value.set(newValue);
         setText(Double.toString(newValue));
      }
      catch (Exception e)
      {
         setText(Double.toString(value.doubleValue()));
      }
   }

   public void setMinValue(double minValue)
   {
      this.minValue = minValue;
      validateValue();
   }

   public void setMaxValue(double maxValue)
   {
      this.maxValue = maxValue;
      validateValue();
   }

   public double getValue()
   {
      return value.doubleValue();
   }

   public void addListener(ChangeListener<? super Number> listener)
   {
      value.addListener(listener);
   }

   public void set(double newValue)
   {
      setText(Double.toString(newValue));
      validateValue();
   }
}

package us.ihmc.parameterTuner.guiElements.tuners;

import org.apache.commons.math3.util.Precision;

import javafx.geometry.Side;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Slider;

public abstract class NumericSlider<T extends Number> extends Slider
{
   public NumericSlider()
   {
      setShowTickMarks(true);
      setShowTickLabels(true);

      // Sets up a context menu that will show the current value while the slider is dragged.
      ContextMenu contextMenu = new ContextMenu();
      MenuItem tooltip = new MenuItem();
      contextMenu.getItems().add(tooltip);
      valueProperty().addListener((observable, oldValue, newValue) -> tooltip.setText(Double.toString(roundToPrecision(getValue()))));
      valueChangingProperty().addListener((observable, oldValue, newValue) -> {
         if (newValue)
         {
            if (!contextMenu.isShowing())
            {
               contextMenu.show(NumericSlider.this, Side.BOTTOM, 0.0, 0.0);
            }
         }
         else
         {
            contextMenu.hide();
         }
      });
   }

   public void updateSlider(T value, T min, T max)
   {
      double doubleValue = doDouble(value);
      double doubleMin = doDouble(min);
      double doubleMax = doDouble(max);

      boolean boundsValid = doubleValue < doubleMin || doubleValue > doubleMax || doubleMin >= doubleMax;
      boolean boundsInfinite = Double.isInfinite(doubleMin) || Double.isInfinite(doubleMax);
      if (boundsValid || boundsInfinite)
      {
         setDisable(true);
         return;
      }

      setDisable(false);
      double range = doubleMax - doubleMin;

      setMin(doubleMin);
      setMax(doubleMax);
      setValue(doubleValue);

      double majorTickUnit = roundToPrecision(range / 2.0);
      majorTickUnit = Math.max(majorTickUnit, getStepSize());
      setMajorTickUnit(majorTickUnit);

      int minorTickCount = (int) (majorTickUnit / getStepSize()) - 1;
      if (minorTickCount > 7)
      {
         setSnapToTicks(false);
         setMinorTickCount(0);
      }
      else
      {
         setSnapToTicks(true);
         setMinorTickCount(minorTickCount);
      }
   }

   public T getNumber()
   {
      return toNumber(roundToPrecision(getValue()));
   }

   public abstract double doDouble(T number);

   public abstract T toNumber(double doubleValue);

   public abstract int getPrecision();

   private double getStepSize()
   {
      return Math.pow(10.0, -getPrecision());
   }

   private double roundToPrecision(double value)
   {
      return Precision.round(value, getPrecision());
   }
}

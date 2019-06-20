package us.ihmc.parameterTuner.guiElements.tuners;

import java.math.BigDecimal;

import javafx.scene.control.SpinnerValueFactory;

public class DoubleSpinnerValueFactory extends SpinnerValueFactory<Double>
{
   private final double increment;
   private double min = Double.NEGATIVE_INFINITY;
   private double max = Double.POSITIVE_INFINITY;

   private final BigDecimal maxDouble = BigDecimal.valueOf(Double.MAX_VALUE);
   private final BigDecimal minDouble = BigDecimal.valueOf(-Double.MAX_VALUE);

   public DoubleSpinnerValueFactory(double increment)
   {
      this.increment = increment;
      valueProperty().addListener((observable, oldValue, newValue) -> {
         if (newValue < min)
         {
            setValue(min);
         }
         if (newValue > max)
         {
            setValue(max);
         }
      });
   }

   public void setMin(Double min)
   {
      this.min = min;
   }

   public void setMax(Double max)
   {
      this.max = max;
   }

   @Override
   public void decrement(int steps)
   {
      if (getValue() == Double.NEGATIVE_INFINITY)
      {
         return;
      }
      else if (getValue() == Double.POSITIVE_INFINITY)
      {
         valueProperty().set(Double.MAX_VALUE);
      }
      else if (Double.isNaN(getValue()))
      {
         valueProperty().set(Double.NaN);
      }
      else
      {
         BigDecimal value = BigDecimal.valueOf(getValue());
         BigDecimal increment = BigDecimal.valueOf(this.increment);
         BigDecimal factor = BigDecimal.valueOf(-steps);
         increment = increment.multiply(factor);
         value = value.add(increment);

         if (value.compareTo(minDouble) < 0 && min == Double.NEGATIVE_INFINITY)
         {
            valueProperty().set(Double.NEGATIVE_INFINITY);
         }
         else
         {
            valueProperty().set(Math.max(min, value.doubleValue()));
         }
      }
   }

   @Override
   public void increment(int steps)
   {
      if (getValue() == Double.POSITIVE_INFINITY)
      {
         return;
      }
      else if (getValue() == Double.NEGATIVE_INFINITY)
      {
         valueProperty().set(-Double.MAX_VALUE);
      }
      else if (Double.isNaN(getValue()))
      {
         valueProperty().set(Double.NaN);
      }
      else
      {
         BigDecimal value = BigDecimal.valueOf(getValue());
         BigDecimal increment = BigDecimal.valueOf(this.increment);
         BigDecimal factor = BigDecimal.valueOf(steps);
         increment = increment.multiply(factor);
         value = value.add(increment);

         if (value.compareTo(maxDouble) > 0 && max == Double.POSITIVE_INFINITY)
         {
            valueProperty().set(Double.POSITIVE_INFINITY);
         }
         else
         {
            valueProperty().set(Math.min(max, value.doubleValue()));
         }
      }
   }
}

package us.ihmc.robotics.parameterGui.tuning;

import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;

public class DoubleSpinner extends NumericSpinner<Double>
{
   public DoubleSpinner()
   {
      super(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
   }

   @Override
   public void setMaxValue(Double maxValue)
   {
      DoubleSpinnerValueFactory valueFactory = (DoubleSpinnerValueFactory) getValueFactory();
      valueFactory.setMax(maxValue);
      revalidate();
   }

   @Override
   public void setMinValue(Double minValue)
   {
      DoubleSpinnerValueFactory valueFactory = (DoubleSpinnerValueFactory) getValueFactory();
      valueFactory.setMin(minValue);
      revalidate();
   }

   @Override
   public Double convertStringToNumber(String numberString)
   {
      return Double.parseDouble(numberString);
   }

   @Override
   public String convertNumberToString(Double number)
   {
      return Double.toString(number);
   }
}

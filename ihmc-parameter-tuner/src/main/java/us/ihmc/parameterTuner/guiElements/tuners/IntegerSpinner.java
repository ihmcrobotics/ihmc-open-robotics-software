package us.ihmc.parameterTuner.guiElements.tuners;

import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import us.ihmc.commons.PrintTools;

public class IntegerSpinner extends NumericSpinner<Integer>
{
   public IntegerSpinner()
   {
      super(new IntegerSpinnerValueFactory(Integer.MIN_VALUE, Integer.MAX_VALUE, 0));
   }

   @Override
   public Integer convertStringToNumber(String numberString)
   {
      if (numberString == null)
      {
         return 0;
      }
      try
      {
         return Integer.parseInt(numberString);
      }
      catch (NumberFormatException e)
      {
         PrintTools.warn("Invalid integer string for IntegerParameter. Attempting to parse as double...");
         double doubleValue = Double.parseDouble(numberString);
         int intValue = (int) doubleValue;

         if (doubleValue != intValue)
         {
            throw new RuntimeException("Integer parameter has floating point precision. Fix your file!");
         }

         return intValue;
      }
   }

   @Override
   public String convertNumberToString(Integer number)
   {
      return Integer.toString(number);
   }
}

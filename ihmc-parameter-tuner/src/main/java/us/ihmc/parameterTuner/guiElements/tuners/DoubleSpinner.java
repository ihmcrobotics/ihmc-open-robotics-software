package us.ihmc.parameterTuner.guiElements.tuners;

public class DoubleSpinner extends NumericSpinner<Double>
{
   public DoubleSpinner()
   {
      super(new DoubleSpinnerValueFactory(0.1));
   }

   @Override
   public Double convertStringToNumber(String numberString)
   {
      if (numberString == null)
      {
         return 0.0;
      }
      if (numberString.endsWith("e") || numberString.endsWith("E"))
      {
         return Double.parseDouble(numberString.substring(0, numberString.length() - 1));
      }
      return Double.parseDouble(numberString);
   }

   @Override
   public String convertNumberToString(Double number)
   {
      return Double.toString(number);
   }

   @Override
   public String[] getSpecialStringOptions()
   {
      return new String[] {convertNumberToString(Double.POSITIVE_INFINITY), convertNumberToString(Double.NEGATIVE_INFINITY)};
   }
}

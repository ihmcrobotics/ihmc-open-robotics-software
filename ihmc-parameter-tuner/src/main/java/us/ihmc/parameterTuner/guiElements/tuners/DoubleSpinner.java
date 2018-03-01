package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

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
   public List<ImmutablePair<String, String>> getSpecialStringOptions()
   {
      List<ImmutablePair<String, String>> ret = new ArrayList<>();
      ret.add(new ImmutablePair<String, String>("Infinity", convertNumberToString(Double.POSITIVE_INFINITY)));
      ret.add(new ImmutablePair<String, String>("Negative Infinity", convertNumberToString(Double.NEGATIVE_INFINITY)));
      ret.add(new ImmutablePair<String, String>("2.0 * PI", convertNumberToString(2.0 * Math.PI)));
      ret.add(new ImmutablePair<String, String>("PI", convertNumberToString(Math.PI)));
      ret.add(new ImmutablePair<String, String>("PI / 2.0", convertNumberToString(Math.PI / 2.0)));
      ret.add(new ImmutablePair<String, String>("PI / 4.0", convertNumberToString(Math.PI / 4.0)));
      ret.add(new ImmutablePair<String, String>("Zero", convertNumberToString(0.0)));
      return ret;
   }
}

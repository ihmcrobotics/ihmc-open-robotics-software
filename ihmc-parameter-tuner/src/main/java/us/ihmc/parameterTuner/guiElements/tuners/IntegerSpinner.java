package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;

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
      return Integer.parseInt(numberString);
   }

   @Override
   public String convertNumberToString(Integer number)
   {
      return Integer.toString(number);
   }

   @Override
   public List<ImmutablePair<String, String>> getSpecialStringOptions()
   {
      return new ArrayList<>();
   }
}

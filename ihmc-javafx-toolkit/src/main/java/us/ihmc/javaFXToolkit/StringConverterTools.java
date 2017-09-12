package us.ihmc.javaFXToolkit;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javafx.scene.control.Slider;
import javafx.util.StringConverter;

/**
 * This class provides simple tools for setting up JavaFX UI control label formatter such as {@link Slider#setLabelFormatter(StringConverter)}. 
 * @author Sylvain Bertrand
 */
public abstract class StringConverterTools
{
   /**
    * @return a StringConverter that converts values expressed in meters to be displayed in centimeters.
    */
   public static StringConverter<Double> metersToRoundedCentimeters()
   {
      return rounding(100.0, 0);
   }

   /**
    * @return a StringConverter that converts values expressed in radians to be displayed in degrees.
    */
   public static StringConverter<Double> radiansToRoundedDegrees()
   {
      return rounding(180.0 / Math.PI, 0);
   }

   /**
    * Provides a simple formatter for large-scale values.
    * @param appendK whether to append the suffix 'k' after the displayed number or not, for instance: {@code 100000} will be displayed {@code 100k} when it is {@code true}.
    * @return reduce large-scale values to display only the thousands, for instance: {@code 100000} will be displayed {@code 100}.
    */
   public static StringConverter<Double> thousandRounding(boolean appendK)
   {
      return rounding(0.001, 0, "k");
   }

   /**
    * Provides a generic {@link StringConverter} that scales then rounds a variable when displaying it.
    * @param toStringScale the scale to apply to values when displaying.
    * @param numberOfDecimals number of displayed decimals before rounding.
    * @return a custom StringConverter.
    */
   public static StringConverter<Double> rounding(double toStringScale, int numberOfDecimals)
   {
      return rounding(toStringScale, numberOfDecimals, "");
   }

   /**
    * Provides a generic {@link StringConverter} that scales then rounds a variable when displaying it.
    * A suffix can be added at the end of the displayed number for instance to clarify the number unit.
    * @param toStringScale the scale to apply to values when displaying.
    * @param numberOfDecimals number of displayed decimals before rounding.
    * @param suffix the suffix to append to the displayed number.
    * @return a custom StringConverter.
    */
   public static StringConverter<Double> rounding(double toStringScale, int numberOfDecimals, String suffix)
   {
      NumberFormat formatter;
      if (numberOfDecimals == 0)
      {
         formatter = new DecimalFormat("0" + suffix + ";-0" + suffix);
      }
      else
      {
         String decimalPart = repeatString("0", numberOfDecimals);
         formatter = new DecimalFormat("0." + decimalPart + suffix + ";-0." + decimalPart + suffix);
      }

      return new StringConverter<Double>()
      {
         @Override
         public String toString(Double object)
         {
            return formatter.format(object * toStringScale);
         }

         @Override
         public Double fromString(String string)
         {
            return Double.parseDouble(string) / toStringScale;
         }
      };
   }

   /**
    * Simple helper method to repeat {@code n} times a given String.
    * @param s the string to repeat.
    * @param n the number of times the stings is to be repeated.
    * @return the repeated string.
    */
   private static String repeatString(String s, int n)
   {
      return new String(new char[n]).replace("\0", s);
   }
}

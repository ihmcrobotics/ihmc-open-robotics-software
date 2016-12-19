package us.ihmc.javaFXToolkit;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javafx.util.StringConverter;

public abstract class StringConverterTools
{
   public static StringConverter<Double> metersToRoundedCentimeters()
   {
      return rounding(100.0, 0);
   }

   public static StringConverter<Double> radiansToRoundedDegrees()
   {
      return rounding(180.0 / Math.PI, 0);
   }

   public static StringConverter<Double> thousandRounding(boolean appendK)
   {
      return rounding(0.001, 0, "k");
   }

   public static StringConverter<Double> rounding(double toStringScale, int numberOfDecimals)
   {
      return rounding(toStringScale, numberOfDecimals, "");
   }

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

   private static String repeatString(String s, int n)
   {
      return new String(new char[n]).replace("\0", s);
   }
}

package us.ihmc.javaFXToolkit;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javafx.util.StringConverter;

public abstract class StringConverterTools
{
   public static StringConverter<Double> metersToRoundedCentimeters()
   {
      return new StringConverter<Double>()
      {
         private final NumberFormat formatter = new DecimalFormat("0;-0");

         @Override
         public String toString(Double object)
         {
            return formatter.format(object * 100.0);
         }
         
         @Override
         public Double fromString(String string)
         {
            return Double.parseDouble(string) / 100.0;
         }
      };
   }
   public static StringConverter<Double> radiansToRoundedDegrees()
   {
      return new StringConverter<Double>()
      {
         private final NumberFormat formatter = new DecimalFormat("0;-0");

         @Override
         public String toString(Double object)
         {
            return formatter.format(Math.toDegrees(object));
         }
         
         @Override
         public Double fromString(String string)
         {
            return Math.toRadians(Double.parseDouble(string));
         }
      };
   }
}

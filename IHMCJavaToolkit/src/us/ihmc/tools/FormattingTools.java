package us.ihmc.tools;

import java.text.DecimalFormat;

import org.apache.commons.lang3.StringUtils;

public class FormattingTools
{
   private static final DecimalFormat decimal3DPrintFormatter = new DecimalFormat("0.000");
   private static final DecimalFormat decimal2DPrintFormatter = new DecimalFormat("0.00");
   private static final DecimalFormat decimal1DPrintFormatter = new DecimalFormat("0.0");

   public static String getFormattedDecimal3D(double value)
   {
      return decimal3DPrintFormatter.format(value);
   }
   
   public static String getFormattedDecimal2D(double value)
   {
      return decimal2DPrintFormatter.format(value);
   }
   
   public static String getFormattedDecimal1D(double value)
   {
      return decimal1DPrintFormatter.format(value);
   }
   
   public static String getFormattedToSignificantFigures(double number, int significantFigures)
   {
      double roundToSignificantFigures = roundToSignificantFigures(number, significantFigures);
      
      if (roundToSignificantFigures >= 10)
         return String.valueOf((int) roundToSignificantFigures);
      else
         return String.valueOf(roundToSignificantFigures);
   }
   
   public static double roundToSignificantFigures(double number, int significantFigures)
   {
      if (number == 0)
      {
         return 0;
      }

      final double d = Math.ceil(Math.log10(number < 0 ? -number : number));
      final int power = significantFigures - (int) d;

      final double magnitude = Math.pow(10, power);
      final long shifted = Math.round(number * magnitude);
      return shifted / magnitude;
   }

   public static String underscoredToCamelCase(String underscoredString, boolean capitalizeFirstLetter)
   {
      char[] charArray = underscoredString.toCharArray();
      boolean capitalizeCurrentLetter = capitalizeFirstLetter;
      int outIndex = 0;
      for (int inIndex = 0; inIndex < charArray.length; inIndex++)
      {
         if (charArray[inIndex] == '_')
         {
            capitalizeCurrentLetter = true;
         }
         else
         {
            char characterIn = charArray[inIndex];
            if (capitalizeCurrentLetter)
            {
               charArray[outIndex++] = Character.toUpperCase(characterIn);
               capitalizeCurrentLetter = false;
            }
            else
            {
               charArray[outIndex++] = Character.toLowerCase(characterIn);
            }
         }
      }
      return new String(charArray, 0, outIndex);
   }

   public static String toHumanReadable(double bits)
   {
      String siPrefix = "";
      
      if(bits > 1024*1024*1024)
      {
         siPrefix = "G";
         bits /= 1024*1024*1024;
      }
      else if(bits > 1024*1024)
      {
         siPrefix = "M";
         bits /= 1024*1024;
      }
      else if(bits > 1024)
      {
         siPrefix = "k";
         bits /= 1024;
      }
      
      return String.format("%.2f", bits) + siPrefix;
   }

   public static String addPrefixAndKeepCamelCaseForMiddleOfExpression(String namePrefix, String originalName)
   {
      return StringUtils.capitalize(addPrefixAndKeepCamelCase(namePrefix, originalName));
   }

   public static String addPrefixAndKeepCamelCase(String namePrefix, String originalName)
   {
      if (namePrefix == null || namePrefix.isEmpty())
         return originalName;
      else if (namePrefix.endsWith("_"))
         return namePrefix + originalName;
      else
         return namePrefix + StringUtils.capitalize(originalName);
   }
}

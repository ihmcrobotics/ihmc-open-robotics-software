package us.ihmc.tools;

import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import org.apache.commons.lang3.StringUtils;

public class FormattingTools
{
   private static final DecimalFormat decimal3DPrintFormatter = new DecimalFormat("0.000");
   private static final DecimalFormat decimal2DPrintFormatter = new DecimalFormat("0.00");
   private static final DecimalFormat decimal1DPrintFormatter = new DecimalFormat("0.0");
   
   private static final SimpleDateFormat dateStringFormatter = new SimpleDateFormat("yyyyMMdd");
   private static final SimpleDateFormat timeStringFormatter = new SimpleDateFormat("HHmm");
   private static final SimpleDateFormat timeStringFormatterWithSeconds = new SimpleDateFormat("HHmmss");

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

      if (Math.abs(roundToSignificantFigures) >= Math.pow(10, significantFigures - 1))
         return String.valueOf((int) roundToSignificantFigures);
      else
         return String.valueOf(roundToSignificantFigures);
   }

   public static double roundToSignificantFigures(double number, int significantFigures)
   {
      if (Math.abs(number) < 1e-10)
      {
         return 0;
      }

      final double log10 = Math.ceil(Math.log10(Math.abs(number)));
      final int power = significantFigures - (int) log10;

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

      if (bits > 1024 * 1024 * 1024)
      {
         siPrefix = "G";
         bits /= 1024 * 1024 * 1024;
      }
      else if (bits > 1024 * 1024)
      {
         siPrefix = "M";
         bits /= 1024 * 1024;
      }
      else if (bits > 1024)
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

   public static String getCommonSuffix(String... strs)
   {
      if (strs == null || strs.length == 0)
      {
         return "";
      }

      String[] reversedStrs = new String[strs.length];
      for (int i = 0; i < strs.length; i++)
         reversedStrs[i] = StringUtils.reverse(strs[i]);
      return StringUtils.reverse(StringUtils.getCommonPrefix(reversedStrs));
   }

   public static String getDateString()
   {
      Date date = new Date();
   
      return dateStringFormatter.format(date);
   }

   public static String getTimeString()
   {
      Date date = new Date();
   
      return timeStringFormatter.format(date);
   }

   public static String getTimeStringWithSeconds()
   {
      Date date = new Date();
   
      return timeStringFormatterWithSeconds.format(date);
   }
}

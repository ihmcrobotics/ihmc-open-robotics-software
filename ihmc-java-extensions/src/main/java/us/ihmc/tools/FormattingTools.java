package us.ihmc.tools;

import java.text.DecimalFormat;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeFormatterBuilder;
import java.time.temporal.ChronoField;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.MathTools;

public class FormattingTools
{
   private static final DecimalFormat decimal3DPrintFormatter = new DecimalFormat("0.000");
   private static final DecimalFormat decimal2DPrintFormatter = new DecimalFormat("0.00");
   private static final DecimalFormat decimal1DPrintFormatter = new DecimalFormat("0.0");

   private static final DateTimeFormatter timeHHMMFormatter;
   private static final DateTimeFormatter timeHHMMSSFormatter;
   static
   {
      DateTimeFormatterBuilder dateTimeFormatterBuilder = new DateTimeFormatterBuilder();
      dateTimeFormatterBuilder.appendValue(ChronoField.HOUR_OF_DAY, 2);
      dateTimeFormatterBuilder.appendValue(ChronoField.MINUTE_OF_HOUR, 2);
      timeHHMMFormatter = dateTimeFormatterBuilder.toFormatter();
      dateTimeFormatterBuilder.appendValue(ChronoField.SECOND_OF_MINUTE, 2);
      timeHHMMSSFormatter = dateTimeFormatterBuilder.toFormatter();
   }

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
      double roundToSignificantFigures = MathTools.roundToSignificantFigures(number, significantFigures);

      if (Math.abs(roundToSignificantFigures) >= Math.pow(10, significantFigures - 1))
         return String.valueOf((int) roundToSignificantFigures);
      else
         return String.valueOf(roundToSignificantFigures);
   }

   public static String getFormattedToPrecision(double value, double precision, int significantFigures)
   {
      double rounded = MathTools.roundToPrecision(value, precision);
      double significant = MathTools.roundToSignificantFigures(rounded, significantFigures);

      if (significant % 1.0 == 0.0)
         return String.valueOf((int) significant);
      else
         return String.valueOf(significant);
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

   /**
    * The date as YYYYMMDD in the system's default timezone.
    * 
    * @see Use {@link java.time.LocalDate}
    * @return The date as YYYYMMDD in the system's default timezone.
    */
   public static String getDateString()
   {
      return LocalDate.now().format(DateTimeFormatter.BASIC_ISO_DATE);
   }

   /**
    * The date as HHMM in the system's default timezone.
    * 
    * @see Use {@link java.time.LocalTime}
    * @return The date as HHMM in the system's default timezone.
    */
   public static String getTimeString()
   {
      return LocalTime.now().format(timeHHMMFormatter);
   }

   /**
    * The date as HHMMSS in the system's default timezone.
    * 
    * @see Use {@link java.time.LocalTime}
    * @return The date as HHMMSS in the system's default timezone.
    */
   public static String getTimeStringWithSeconds()
   {
      return LocalTime.now().format(timeHHMMSSFormatter);
   }
}

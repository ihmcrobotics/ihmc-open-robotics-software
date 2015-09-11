package us.ihmc.tools;

import java.text.DecimalFormat;

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

   /**
    * Use StringUtils.captilize
    * @param s
    * @return
    */
   @Deprecated
   public static String capitalizeFirstLetter(String s)
   {
      if (s.length() == 0)
         return s;

      return s.substring(0, 1).toUpperCase() + s.substring(1);
   }
   
   /**
    * Use StringUtils.uncaptilize
    * @param s
    * @return
    */
   @Deprecated
   public static String lowerCaseFirstLetter(String s)
   {
      if (s.length() == 0)
         return s;

      return s.substring(0, 1).toLowerCase() + s.substring(1);
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
   
   final protected static char[] hexArray = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',' '};
   public static String bytesToHex(byte[] bytes) {
       char[] hexChars = new char[bytes.length * 3 -1];
       int v;
       for ( int j = 0; j < bytes.length; j++ ) {
           v = bytes[j] & 0xFF;
           hexChars[j * 3] = hexArray[v >>> 4];
           hexChars[j * 3 + 1] = hexArray[v & 0x0F];
           if (j<bytes.length-1)hexChars[j * 3 + 2] = hexArray[16];
       }
       return new String(hexChars);
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
      return capitalizeFirstLetter(addPrefixAndKeepCamelCase(namePrefix, originalName));
   }

   public static String addPrefixAndKeepCamelCase(String namePrefix, String originalName)
   {
      if (namePrefix == null || namePrefix.isEmpty())
         return originalName;
      else if (namePrefix.endsWith("_"))
         return namePrefix + originalName;
      else
         return namePrefix + capitalizeFirstLetter(originalName);
   }
}

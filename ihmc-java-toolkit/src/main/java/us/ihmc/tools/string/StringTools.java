package us.ihmc.tools.string;

import org.apache.commons.lang3.StringUtils;
import org.apache.logging.log4j.message.ParameterizedMessageFactory;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.function.Supplier;
import java.util.regex.Pattern;

public class StringTools
{
   private StringTools()
   {
      // disallow construction
   }

   public static String getEveryUppercaseLetter(String string)
   {
      String ret = "";
      int numberOfCharaters = string.length();
      for(int i = 0; i < numberOfCharaters; i++){
         if(Character.isUpperCase(string.charAt(i))){
            char characterToAdd = string.charAt(i);
            ret = ret + characterToAdd;
         }
      }

      return ret;
   }

   public static Supplier<String> format(String message, Object... parameters)
   {
      return () -> ParameterizedMessageFactory.INSTANCE.newMessage(message, parameters).getFormattedMessage();
   }

   /**
    * Format number to 3 decimal places.
    */
   public static Supplier<String> format3D(String message, Object... parameters)
   {
      Object[] formattedParameters = new Object[parameters.length];
      for (int i = 0; i < parameters.length; i++)
      {
         if (parameters[i] instanceof Double)
         {
            formattedParameters[i] = new FormattedDouble3D((Double) parameters[i]);
         }
         else
         {
            formattedParameters[i] = parameters[i];
         }
      }

      return format(message, formattedParameters);
   }

   /**
    * Format numbers to 3 significant figures.
    */
   public static Supplier<String> format3SF(String message, Object... parameters)
   {
      Object[] formattedParameters = new Object[parameters.length];
      for (int i = 0; i < parameters.length; i++)
      {
         if (parameters[i] instanceof Double)
         {
            formattedParameters[i] = new FormattedDouble3SF((Double) parameters[i]);
         }
         else
         {
            formattedParameters[i] = parameters[i];
         }
      }

      return format(message, formattedParameters);
   }

   public static String zUpPoseString(Pose3DReadOnly pose)
   {
      return "(" + FormattingTools.getFormattedDecimal3D(pose.getX())
            + ", " + FormattingTools.getFormattedDecimal3D(pose.getY())
            + ", " + FormattingTools.getFormattedDecimal3D(pose.getZ())
            + ") yaw: " + FormattingTools.getFormattedDecimal3D(pose.getYaw());
   }

   public static String tupleString(Tuple3DReadOnly tuple)
   {
      return "(" + FormattingTools.getFormattedDecimal3D(tuple.getX())
             + ", " + FormattingTools.getFormattedDecimal3D(tuple.getY())
             + ", " + FormattingTools.getFormattedDecimal3D(tuple.getZ())
             + ")";
   }

   public static class FormattedDouble3D
   {
      private final Double number;

      public FormattedDouble3D(Double number)
      {
         this.number = number;
      }

      @Override
      public String toString()
      {
         return FormattingTools.getFormattedDecimal3D(number);
      }
   }

   public static class FormattedDouble3SF
   {
      private final Double number;

      public FormattedDouble3SF(Double number)
      {
         this.number = number;
      }

      @Override
      public String toString()
      {
         return FormattingTools.getFormattedToSignificantFigures(number, 3);
      }
   }

   public static String titleToSnakeCase(String titleCased)
   {
      return titleCased.trim().replaceAll("\\s+", "_").toLowerCase();
   }

   public static String titleToKebabCase(String titleCased)
   {
      return titleCased.trim().replaceAll("\\s+", "-").toLowerCase();
   }

   public static String titleToPascalCase(String titleCased)
   {
      String[] parts = titleCased.split("\\s+");
      StringBuilder camelCased = new StringBuilder();
      for (int i = 0; i < parts.length; i++)
      {
         camelCased.append(StringUtils.capitalize(parts[i]));
      }
      return camelCased.toString();
   }

   public static String titleToCamelCase(String titleCased)
   {
      String[] parts = titleCased.split("\\s+");
      StringBuilder camelCased = new StringBuilder();
      for (int i = 0; i < parts.length; i++)
      {
         if (i == 0)
            camelCased.append(parts[i].toLowerCase());
         else
            camelCased.append(StringUtils.capitalize(parts[i]));
      }
      return camelCased.toString();
   }

   private static final Pattern UPPERCASE_LETTER = Pattern.compile("(?<=[a-z])[A-Z]+|[A-Z](?=[a-z])|[0-9]+");

   /**
    * PascalCase to sentence case. Can handle acronyms in some cases, but does struggle sometimes.
    * @param pascalCaseString the pascal case string
    * @return the sentence case string
    */
   public static String pascalCaseToSentenceCase(String pascalCaseString) {
      return pascalCaseString.substring(0, 1).toUpperCase() + UPPERCASE_LETTER.matcher(pascalCaseString.substring(1))
                                                                             .replaceAll(matchResult -> !matchResult.group().matches("[A-Z]{2,}")
                                                                                   ? " " + matchResult.group().toLowerCase()
                                                                                   : " " + matchResult.group());
   }

   /**
    * Useful for parsing files handling multiple operating systems.
    */
   public static String filterOutCRLFLineEndings(String stringToFilter)
   {
      return stringToFilter.replaceAll("\\r\\n", "\n");
   }
}

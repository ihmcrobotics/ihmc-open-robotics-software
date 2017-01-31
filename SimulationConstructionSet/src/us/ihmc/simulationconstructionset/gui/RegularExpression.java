package us.ihmc.simulationconstructionset.gui;

import java.util.StringTokenizer;
import java.util.regex.PatternSyntaxException;

public class RegularExpression
{
   public static String previousSearch = "";

   public static boolean check(String checkString, String regex)
   {
      regex = regex.replaceAll(" ", "");
      checkString = checkString.toLowerCase();
      regex = regex.toLowerCase();

      if (regex.contains("&"))
      {
         return performAndSearch(checkString, regex);
      }

      if (regex.contains("|"))
      {
         regex = builtNewDelimitedString(checkString, regex, "|");
      }

      if (!regex.contains("*") &&!(regex.startsWith("\"") && regex.endsWith("\"")))
      {
         regex = addStars(regex);
      }

      if (regex.startsWith("\"") && regex.endsWith("\""))
      {
         regex = regex.replaceAll("\"", "");

         return checkString.equals(regex);
      }




      if (!regex.startsWith("*")&& !regex.contains("*"))
      {
         regex = "*" + regex;
      }

      if (!regex.endsWith("*")&& !regex.contains("*"))
      {
         regex = regex + "*";
      }

      return regChecker(checkString, regex) || startsWithChecker(checkString, regex);
   }

   private static boolean performAndSearch(String checkString, String regex)
   {
      StringTokenizer tok = new StringTokenizer(regex, "&");
      int tokenCount = tok.countTokens();
      for (int i = 0; i < tokenCount; i++)
      {
         String nextToken = tok.nextToken();
         if (!(check(checkString, nextToken)))
            return false;
      }

      return true;
   }

   private static String addStars(String checkString)
   {
      return "*" + checkString + "*";
   }

   private static String builtNewDelimitedString(String checkString, String regex, String delem)
   {
      StringTokenizer tok = new StringTokenizer(regex, delem);

      String finalString = "";
      int tokenCount = tok.countTokens();
      for (int i = 0; i < tokenCount; i++)
      {
         String nextToken = tok.nextToken();

         if (!nextToken.contains("*") &&!(nextToken.startsWith("\"") && nextToken.endsWith("\"")))
         {
            nextToken = addStars(nextToken);
         }

         finalString += nextToken;
         if (i < tokenCount - 1)
            finalString += delem;
      }

      return finalString;
   }

   private static boolean startsWithChecker(String checkString, String regex)
   {
      return checkString.startsWith(regex);
   }

   private static boolean regChecker(String checkString, String regex)
   {
      StringTokenizer t = new StringTokenizer(regex, "*");
      String finalString = "";
      int tokenCount = t.countTokens();
      for (int i = 0; i < tokenCount; i++)
      {
         finalString += t.nextToken();
         if (i < tokenCount - 1)
            finalString += ".*";
      }

      if (regex.startsWith("*"))
         finalString = ".*" + finalString;
      if (regex.endsWith("*"))
         finalString += ".*";

      boolean matches = false;
      try
      {
         matches = checkString.matches(finalString);
         previousSearch = finalString;
      }
      catch (PatternSyntaxException e)
      {
         matches = checkString.matches(previousSearch);
      }

      return matches;
   }

   public static void main(String[] args)
   {
      System.out.println(RegularExpression.check("testsomethingtorque", "something*torque*"));
   }
}

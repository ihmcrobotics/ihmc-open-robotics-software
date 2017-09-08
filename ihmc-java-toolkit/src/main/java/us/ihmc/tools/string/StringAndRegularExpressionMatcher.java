package us.ihmc.tools.string;

import java.util.ArrayList;
import java.util.regex.Pattern;

public class StringAndRegularExpressionMatcher
{
   private final ArrayList<String> exactStringsToMatch = new ArrayList<String>();
   private final ArrayList<Pattern> regularExpressionsToMatch = new ArrayList<Pattern>();
   
   private final boolean isCaseSensitive;
   
   public StringAndRegularExpressionMatcher()
   {
      this(true);
   }
   
   public StringAndRegularExpressionMatcher(boolean isCaseSensitive)
   {
      this.isCaseSensitive = isCaseSensitive;
   }
   
   public void addExactStringToMatch(String exactStringToMatch)
   {
      if (!isCaseSensitive)
      {
         exactStringToMatch = exactStringToMatch.toLowerCase();
      }
      
      this.exactStringsToMatch.add(exactStringToMatch);
   }
   
   public void addRegularExpression(String regularExpression)
   {
      Pattern pattern = Pattern.compile(regularExpression);
      this.regularExpressionsToMatch.add(pattern);
   }

   public boolean matches(String stringToTest)
   {
      if (!isCaseSensitive)
      {
         stringToTest = stringToTest.toLowerCase();
      }
      
      if (this.exactStringsToMatch.contains(stringToTest)) return true;

      for (Pattern pattern : regularExpressionsToMatch)
      {
         if (pattern.matcher(stringToTest).matches())
         {
            return true;
         }
      }
      
      return false;
   }
}

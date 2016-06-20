package us.ihmc.tools.search.strings.fuzzySearch;

import java.util.Arrays;
import java.util.Collection;
import java.util.Locale;

/**
 * Enum type used to indicate what "category" a search result from the
 * CombinedFuzzySearcher came from.
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class CombinedFuzzySearchResult
{
   private final CombinedFuzzySearchResultType resultType;
   private final String[] subStringMatches;
   private final String searchMatch;
   private final int fuzzyScore;

   public CombinedFuzzySearchResult(String searchMatch, CombinedFuzzySearchResultType resultType, int fuzzyScore, String... subStringMatches)
   {
      this.resultType = resultType;
      this.subStringMatches = subStringMatches;
      this.searchMatch = searchMatch;
      this.fuzzyScore = fuzzyScore;
   }

   public CombinedFuzzySearchResult(String searchMatch, CombinedFuzzySearchResultType resultType, String... subStringMatches)
   {
      this(searchMatch, resultType, -1, subStringMatches);
   }

   public CombinedFuzzySearchResultType getResultType()
   {
      return resultType;
   }

   public String[] getSubStringMatches()
   {
      return subStringMatches;
   }

   public String getSearchMatch()
   {
      return searchMatch;
   }

   public int getFuzzyScore()
   {
      return fuzzyScore;
   }

   @Override
   public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      CombinedFuzzySearchResult that = (CombinedFuzzySearchResult) o;

      if (fuzzyScore != that.fuzzyScore)
         return false;
      if (resultType != that.resultType)
         return false;

      if (!Arrays.equals(subStringMatches, that.subStringMatches))
         return false;
      return searchMatch.equals(that.searchMatch);

   }

   @Override
   public int hashCode()
   {
      int result = resultType.hashCode();
      result = 31 * result + Arrays.hashCode(subStringMatches);
      result = 31 * result + searchMatch.hashCode();
      result = 31 * result + fuzzyScore;
      return result;
   }

   public enum CombinedFuzzySearchResultType
   {
      /**
       * FUZZY results are based on the Apache Commons Lang library's
       * getFuzzyDistance() method.
       *
       * @see org.apache.commons.lang3.StringUtils#getFuzzyDistance(CharSequence, CharSequence, Locale)
       */
      FUZZY,
      /**
       * REGEX results are based on a simple regular expression matcher using the built-in Java
       * regex engine.
       *
       * @see java.util.regex.Pattern
       * @see CombinedFuzzySearcher#performRegularExpressionSearch(String, Collection)
       */
      REGEX,
      /**
       * EXACT_SUBSTRING results are the results of matching {@link String#contains(CharSequence)}
       */
      EXACT_SUBSTRING
   }
}

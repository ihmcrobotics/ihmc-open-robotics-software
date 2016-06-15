package us.ihmc.tools.search.strings.fuzzySearch;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.search.strings.fuzzySearch.comparators.SortByMatchedStringComparator;
import us.ihmc.tools.search.strings.fuzzySearch.comparators.SortByResultTypeComparator;
import us.ihmc.tools.search.strings.fuzzySearch.comparators.SortFuzzyMatchesDescendingByFuzzyDistanceComparator;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class CombinedFuzzySearcher
{
   private CombinedFuzzySearcher()
   {
   }

   public static List<CombinedFuzzySearchResult> getAllSearchResultsForSearchString(Collection<String> stringsToSearch, String searchString)
   {
      Set<CombinedFuzzySearchResult> resultsOfFuzzyDistanceSearch;
      Set<CombinedFuzzySearchResult> resultsOfRegexSearch;
      Set<CombinedFuzzySearchResult> resultsOfExactSubstringSearch;

      resultsOfFuzzyDistanceSearch = performFuzzyStringSearch(searchString, stringsToSearch);

      resultsOfRegexSearch = performRegularExpressionSearch(searchString, stringsToSearch);

      resultsOfExactSubstringSearch = performExactSubstringSearch(searchString, stringsToSearch);

      TreeSet<CombinedFuzzySearchResult> accumulator = new TreeSet<>(new SortByMatchedStringComparator());

      accumulator.addAll(resultsOfExactSubstringSearch);
      accumulator.addAll(resultsOfRegexSearch);

      List<CombinedFuzzySearchResult> allSearchResults = new ArrayList<>(accumulator);
      allSearchResults.sort(new SortByResultTypeComparator());

      ArrayList<CombinedFuzzySearchResult> sortedFuzzyResults = new ArrayList<>();

      for (CombinedFuzzySearchResult combinedFuzzySearchResult : resultsOfFuzzyDistanceSearch)
      {
         if (!accumulator.contains(combinedFuzzySearchResult))
         {
            sortedFuzzyResults.add(combinedFuzzySearchResult);
         }
      }

      Collections.sort(sortedFuzzyResults, new SortFuzzyMatchesDescendingByFuzzyDistanceComparator());

      return allSearchResults;
   }

   private static Set<CombinedFuzzySearchResult> performFuzzyStringSearch(String searchString, Collection<String> stringsToSearch)
   {
      Set<CombinedFuzzySearchResult> resultsOfFuzzyDistanceSearch = new HashSet<>();

      for (String stringToSearch : stringsToSearch)
      {
         int fuzzyDistance = StringUtils.getFuzzyDistance(stringToSearch, searchString, Locale.ENGLISH);
         if (fuzzyDistance > 0)
         {
            resultsOfFuzzyDistanceSearch
                  .add(new CombinedFuzzySearchResult(stringToSearch, CombinedFuzzySearchResult.CombinedFuzzySearchResultType.FUZZY, fuzzyDistance,
                        searchString.split("")));
         }
      }

      TreeSet<CombinedFuzzySearchResult> sortedResults = new TreeSet<>(new SortFuzzyMatchesDescendingByFuzzyDistanceComparator());

      sortedResults.addAll(resultsOfFuzzyDistanceSearch);

      return sortedResults;
   }

   private static Set<CombinedFuzzySearchResult> performRegularExpressionSearch(String searchString, Collection<String> stringsToSearch)
   {
      Set<CombinedFuzzySearchResult> resultsOfRegexSearch = new HashSet<>();
      try
      {
         String wildCardBoundedRegexSearchString = "";

         if (!searchString.startsWith("^"))
         {
            wildCardBoundedRegexSearchString += "(?<startGroup>.*)";
         }

         wildCardBoundedRegexSearchString += searchString;

         if (!searchString.endsWith("$"))
         {
            wildCardBoundedRegexSearchString += "(?<endGroup>.*)";
         }

         boolean hasStartGroup = !wildCardBoundedRegexSearchString.startsWith("^");
         boolean hasEndGroup = !wildCardBoundedRegexSearchString.endsWith("$");
         Pattern simpleRegexPattern = Pattern.compile(wildCardBoundedRegexSearchString, Pattern.CASE_INSENSITIVE);

         for (String stringToSearch : stringsToSearch)
         {
            if(simpleRegexPattern.matcher(stringToSearch).matches())
            {
               resultsOfRegexSearch.add(getSearchResultFromRegex(hasStartGroup, hasEndGroup, simpleRegexPattern, stringToSearch));
            }
         }
      }
      catch (PatternSyntaxException ignored)
      {
         resultsOfRegexSearch = Collections.emptySet();
      }
      return resultsOfRegexSearch;
   }

   private static Set<CombinedFuzzySearchResult> performExactSubstringSearch(String searchString, Collection<String> stringsToSearch)
   {
      Set<CombinedFuzzySearchResult> resultsOfExactSubstringSearch = new HashSet<>();

      for (String stringToSearch : stringsToSearch)
      {
         if(searchString.contains(searchString))
         {
            resultsOfExactSubstringSearch.add(new CombinedFuzzySearchResult(stringToSearch, CombinedFuzzySearchResult.CombinedFuzzySearchResultType.EXACT_SUBSTRING, searchString));
         }
      }

      return resultsOfExactSubstringSearch;
   }

   private static CombinedFuzzySearchResult getSearchResultFromRegex(boolean hasStartGroup, boolean hasEndGroup, Pattern simpleRegexPattern, String stringToSearch)
   {
      ArrayList<String> matches = new ArrayList<>();
      Matcher matcher = simpleRegexPattern.matcher(stringToSearch);

      while (matcher.find())
      {
         String group = matcher.group();

         if (hasStartGroup)
         {
            String startGroup = matcher.group("startGroup");
            if (group.startsWith(startGroup))
            {
               group = group.replace(startGroup, "");
            }
         }

         if (hasEndGroup)
         {
            String endGroup = matcher.group("endGroup");
            if (group.endsWith(endGroup))
            {
               group = group.replace(endGroup, "");
            }
         }

         matches.add(group);
      }

      return new CombinedFuzzySearchResult(stringToSearch, CombinedFuzzySearchResult.CombinedFuzzySearchResultType.REGEX, matches.toArray(new String[matches.size()]));
   }
}

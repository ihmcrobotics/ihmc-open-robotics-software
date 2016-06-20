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

   /**
    * <p>
    * This method uses several search techniques simultaneously to heuristically build up a list
    * of possible matches based on the passed in search string. It searches the supplied collection
    * for all possible matches.
    * </p>
    *
    * <p>
    * Three searches will be performed: Matching on the exact substring that is passed in, matching on
    * regular expression, and using a "fuzzy" search similar to many popular text editors and IDEs that
    * support fuzzy searching w/out the need for wildcards (Atom, Sublime Text, IntelliJ IDEA, etc.) provided
    * by Apache Commons Lang.
    * </p>
    *
    * <p>
    * The returned List is sorted and has element uniqueness based on the following set of criteria:
    * <ol>
    * <li> Any given String, for its fully qualified name, will only show up once, and,</li>
    * <li> The matching information for that String will correspond to search "priority"</li>
    * </ol>
    *</p>
    *
    * <p>
    * In this case, priority simply means that if the same String is identified by multiple
    * search techniques (e.g. both Exact Substring and Fuzzy Search find the same String for the same query), then only
    * the {@link CombinedFuzzySearchResult} with {@link us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult.CombinedFuzzySearchResultType} corresponding to the highest
    * priority will be in the result set. The priorities are:
    *
    * <ol>
    * <li> Exact Substring has highest priority</li>
    * <li> Regular Expression is next</li>
    * <li> Fuzzy results are last</li>
    * </ol>
    * </p>
    *
    * @see CombinedFuzzySearchResult
    * @see us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult.CombinedFuzzySearchResultType
    * @see StringUtils#getFuzzyDistance(CharSequence, CharSequence, Locale)
    * @see Pattern
    * @see Matcher
    *
    * @param stringsToSearch The registry to search. Searches will be recursive and will still utilize the fully qualified name even if this is not the root registry.
    * @param searchString The query to search for.
    *
    * @return A list of unique {@link CombinedFuzzySearchResult}s sorted according to their {@link us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult.CombinedFuzzySearchResultType} priority.
    */
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
      Collections.sort(allSearchResults, new SortByResultTypeComparator());

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

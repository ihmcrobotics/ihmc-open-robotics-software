package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult.CombinedFuzzySearchResultType;

import java.util.Comparator;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortByResultTypeComparator implements Comparator<CombinedFuzzySearchResult>
{
   @Override
   public int compare(CombinedFuzzySearchResult o1, CombinedFuzzySearchResult o2)
   {
      CombinedFuzzySearchResultType firstResultType = o1.getResultType();
      CombinedFuzzySearchResultType secondResultType = o2.getResultType();
      if (firstResultType == secondResultType)
         return 0;

      if (firstResultType == CombinedFuzzySearchResultType.EXACT_SUBSTRING)
         return -1;
      else if (secondResultType == CombinedFuzzySearchResultType.EXACT_SUBSTRING)
         return 1;
      else if (firstResultType == CombinedFuzzySearchResultType.REGEX && secondResultType == CombinedFuzzySearchResultType.FUZZY)
         return -1;
      else
         return 1;
   }
}

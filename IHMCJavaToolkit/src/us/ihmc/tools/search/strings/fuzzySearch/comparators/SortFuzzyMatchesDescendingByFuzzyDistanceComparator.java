package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;

import java.util.Comparator;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortFuzzyMatchesDescendingByFuzzyDistanceComparator implements Comparator<CombinedFuzzySearchResult>
{
   @Override
   public int compare(CombinedFuzzySearchResult o1, CombinedFuzzySearchResult o2)
   {
      return Integer.compare(o2.getFuzzyScore(), o1.getFuzzyScore());
   }
}

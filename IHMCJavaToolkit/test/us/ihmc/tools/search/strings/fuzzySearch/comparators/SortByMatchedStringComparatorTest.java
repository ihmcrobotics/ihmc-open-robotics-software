package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import org.junit.Test;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortByMatchedStringComparatorTest
{

   @Test
   public void testCompare()
   {
      CombinedFuzzySearchResult aardvark = new CombinedFuzzySearchResult("Aardvark", CombinedFuzzySearchResult.CombinedFuzzySearchResultType.EXACT_SUBSTRING,
            null);

      CombinedFuzzySearchResult sebastopol = new CombinedFuzzySearchResult("Sebastopol", CombinedFuzzySearchResult.CombinedFuzzySearchResultType.EXACT_SUBSTRING,
            null);

      SortByMatchedStringComparator comparator = new SortByMatchedStringComparator();

      assertTrue(comparator.compare(aardvark, sebastopol) < 0);
      assertTrue(comparator.compare(sebastopol, aardvark) > 0);
      assertEquals(0, comparator.compare(aardvark, aardvark));
      assertEquals(0, comparator.compare(sebastopol, sebastopol));
   }
}
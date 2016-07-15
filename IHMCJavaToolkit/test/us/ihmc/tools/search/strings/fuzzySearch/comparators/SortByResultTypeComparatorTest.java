package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import org.junit.Test;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult.CombinedFuzzySearchResultType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

import static org.junit.Assert.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortByResultTypeComparatorTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCompare()
   {
      Random random = new Random(1976L);
      SortByResultTypeComparator comparator = new SortByResultTypeComparator();

      CombinedFuzzySearchResult aardvark = new CombinedFuzzySearchResult("Aardvark", CombinedFuzzySearchResultType.EXACT_SUBSTRING, null);
      CombinedFuzzySearchResult burgundy = new CombinedFuzzySearchResult("Burgundy", CombinedFuzzySearchResultType.REGEX, null);
      CombinedFuzzySearchResult sebastopol = new CombinedFuzzySearchResult("Sebastopol", CombinedFuzzySearchResultType.FUZZY, null);

      for (int i = 0; i < 10000; i++)
      {
         CombinedFuzzySearchResultType randomResultType = generateRandomResultType(random);
         CombinedFuzzySearchResult victorious = new CombinedFuzzySearchResult("Victorious", randomResultType, null);

         int exactSubstringCompare = comparator.compare(aardvark, victorious);
         int regexCompare = comparator.compare(burgundy, victorious);
         int fuzzyCompare = comparator.compare(sebastopol, victorious);

         switch (randomResultType)
         {
         case FUZZY:
            assertTrue(exactSubstringCompare < 0);
            assertTrue(regexCompare < 0);
            assertTrue(fuzzyCompare == 0);
            break;
         case REGEX:
            assertTrue(exactSubstringCompare < 0);
            assertTrue(regexCompare == 0);
            assertTrue(fuzzyCompare > 0);
            break;
         case EXACT_SUBSTRING:
            assertTrue(exactSubstringCompare == 0);
            assertTrue(regexCompare > 0);
            assertTrue(fuzzyCompare > 0);
            break;
         }
      }

   }

   private static CombinedFuzzySearchResultType generateRandomResultType(Random random)
   {
      int numberOfEnums = CombinedFuzzySearchResultType.class.getEnumConstants().length;
      return CombinedFuzzySearchResultType.class.getEnumConstants()[random.nextInt(numberOfEnums)];
   }
}
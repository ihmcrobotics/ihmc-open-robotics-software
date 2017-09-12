package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortFuzzyMatchesDescendingByFuzzyDistanceComparatorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCompare()
   {
      Random random = new Random(1976L);
      for (int i = 0; i < 10000; i++)
      {
         int firstRandomDistance = random.nextInt(300) + 1;
         int secondRandomDistance = random.nextInt(300) + 1;

         CombinedFuzzySearchResult aardvark = new CombinedFuzzySearchResult("Aardvark", CombinedFuzzySearchResult.CombinedFuzzySearchResultType.FUZZY,
               firstRandomDistance);

         CombinedFuzzySearchResult sebastopol = new CombinedFuzzySearchResult("Sebastopol", CombinedFuzzySearchResult.CombinedFuzzySearchResultType.FUZZY,
               secondRandomDistance);

         SortFuzzyMatchesDescendingByFuzzyDistanceComparator comparator = new SortFuzzyMatchesDescendingByFuzzyDistanceComparator();

         int compare = comparator.compare(aardvark, sebastopol);

         if(firstRandomDistance > secondRandomDistance)
         {
            assertTrue(compare < 0);
         }
         else if (firstRandomDistance < secondRandomDistance)
         {
            assertTrue(compare > 0);
         }
         else if(firstRandomDistance == secondRandomDistance)
         {
            assertTrue(compare == 0);
         }
      }
   }
}
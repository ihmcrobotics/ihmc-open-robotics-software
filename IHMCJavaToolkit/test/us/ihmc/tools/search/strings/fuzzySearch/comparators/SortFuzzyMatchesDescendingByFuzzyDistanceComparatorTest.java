package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import org.junit.Test;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

import static org.junit.Assert.assertTrue;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortFuzzyMatchesDescendingByFuzzyDistanceComparatorTest
{
   @DeployableTestMethod(estimatedDuration = 0.01)
   @Test(timeout = 100)
   public void testCompare()
   {
      Random random = new Random(1976L);
      for (int i = 0; i < 10000; i++)
      {
         int firstRandomDistance = random.nextInt(300) + 1;
         int secondRandomDistance = random.nextInt(300) + 1;

         CombinedFuzzySearchResult aardvark = new CombinedFuzzySearchResult("Aardvark", CombinedFuzzySearchResult.CombinedFuzzySearchResultType.FUZZY,
               firstRandomDistance, null);

         CombinedFuzzySearchResult sebastopol = new CombinedFuzzySearchResult("Sebastopol", CombinedFuzzySearchResult.CombinedFuzzySearchResultType.FUZZY,
               secondRandomDistance, null);

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
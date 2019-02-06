package us.ihmc.tools.search.strings.fuzzySearch.comparators;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.tools.search.strings.fuzzySearch.CombinedFuzzySearchResult;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SortFuzzyMatchesDescendingByFuzzyDistanceComparatorTest
{
   @Test
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
package us.ihmc.tools.search.strings.fuzzySearch;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@ContinuousIntegrationPlan(targets = TestPlanTarget.Exclude)
public class CombinedFuzzySearcherTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testGetAllSearchResultsForSearchString()
   {

   }
}
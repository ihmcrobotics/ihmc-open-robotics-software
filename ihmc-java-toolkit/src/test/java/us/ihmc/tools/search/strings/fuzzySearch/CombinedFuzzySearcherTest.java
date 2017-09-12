package us.ihmc.tools.search.strings.fuzzySearch;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class CombinedFuzzySearcherTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testGetAllSearchResultsForSearchString()
   {

   }
}
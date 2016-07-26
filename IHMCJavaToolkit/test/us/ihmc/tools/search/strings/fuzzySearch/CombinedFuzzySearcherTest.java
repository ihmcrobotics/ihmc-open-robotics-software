package us.ihmc.tools.search.strings.fuzzySearch;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@DeployableTestClass(targets = TestPlanTarget.Exclude)
public class CombinedFuzzySearcherTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testGetAllSearchResultsForSearchString()
   {

   }
}
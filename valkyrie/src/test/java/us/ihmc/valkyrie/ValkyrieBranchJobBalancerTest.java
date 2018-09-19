package us.ihmc.valkyrie;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ValkyrieBranchJobBalancerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 180.0)
   @Test(timeout = 30000)
   public void testExtraTimeToSyncJobs()
   {

   }
}

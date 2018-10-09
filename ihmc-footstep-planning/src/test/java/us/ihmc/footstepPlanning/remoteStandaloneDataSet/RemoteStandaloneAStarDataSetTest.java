package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class RemoteStandaloneAStarDataSetTest extends RemoteStandalonePlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }


   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 125.0)
   public void testDatasetsWithoutOcclusion()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 13.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }

   public static void main(String[] args) throws Exception
   {
      RemoteStandaloneAStarDataSetTest test = new RemoteStandaloneAStarDataSetTest();
      String prefix = "unitTestData/testable/";
      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), prefix + "20171026_131304_PlanarRegion_Ramp_2Story_UnitTest");
      test.tearDown();

   }
}

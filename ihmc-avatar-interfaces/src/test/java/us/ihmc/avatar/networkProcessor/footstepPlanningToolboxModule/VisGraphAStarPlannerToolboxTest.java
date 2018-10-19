package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class VisGraphAStarPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 85.0)
   public void testDatasetsWithoutOcclusion()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 13.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(dataset -> runAssertions(dataset));
   }

   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 4.2, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   public static void main(String[] args) throws Exception
   {
      VisGraphAStarPlannerToolboxTest test = new VisGraphAStarPlannerToolboxTest();
      String prefix = "unitTestDataSets/test/";
      VISUALIZE = true;
      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_205120_BodyPathPlannerEnvironment");
      PrintTools.info("Test passed.");
      test.tearDown();
   }
}

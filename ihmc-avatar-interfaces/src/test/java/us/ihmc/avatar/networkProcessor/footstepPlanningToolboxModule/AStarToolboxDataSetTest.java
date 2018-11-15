package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerDataExporter;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.pubsub.DomainFactory;

import java.util.List;

import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.MANUAL)
public class AStarToolboxDataSetTest extends FootstepPlannerToolboxDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 2.5)
   @Test(timeout = 1000000)
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }


   public static void main(String[] args) throws Exception
   {
      AStarToolboxDataSetTest test = new AStarToolboxDataSetTest();
      String prefix = "unitTestDataSets/test/";

      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_204917_FlatGround");

      ThreadTools.sleepForever();
      test.tearDown();
      PrintTools.info("Test passed.");
   }
}

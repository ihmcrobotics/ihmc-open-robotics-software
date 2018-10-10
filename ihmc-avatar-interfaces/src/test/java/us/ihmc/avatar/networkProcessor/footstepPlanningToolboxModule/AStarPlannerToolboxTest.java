package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   public static void main(String[] args) throws Exception
   {
      AStarPlannerToolboxTest test = new AStarPlannerToolboxTest();
      String prefix = "unitTestData/testable/";
      test.pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      test.setup();
//      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), prefix + "20171215_214730_CinderBlockField");
      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), prefix + "20171215_220523_SteppingStones");
      PrintTools.info("Test passed.");
      test.tearDown();
   }
}

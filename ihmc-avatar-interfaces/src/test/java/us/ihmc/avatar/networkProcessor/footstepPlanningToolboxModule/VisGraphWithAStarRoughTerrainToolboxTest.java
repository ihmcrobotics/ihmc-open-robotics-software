package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class VisGraphWithAStarRoughTerrainToolboxTest extends RoughTerrainDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 2.5)
   @Test(timeout = 1000000)
   public void testDownCorridor()
   {
      setCheckForBodyBoxCollision(true);
      super.testDownCorridor();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 2.5)
   @Test(timeout = 1000000)
   public void testBetweenTwoBollards()
   {
      setCheckForBodyBoxCollision(true);
      setBodyBoxDepth(0.45);
      setBodyBoxWidth(0.9);
      setBodyBoxOffsetX(0.1);
      super.testBetweenTwoBollards();
   }

   public static void main(String[] args) throws Exception
   {
      String testName = corridor;
      VisGraphWithAStarRoughTerrainToolboxTest test = new VisGraphWithAStarRoughTerrainToolboxTest();
      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      VISUALIZE = true;
      test.setup();
      test.setCheckForBodyBoxCollision(true);

      if (testName.equals(bollards))
      {
         test.setBodyBoxDepth(0.45);
         test.setBodyBoxWidth(0.9);
         test.setBodyBoxOffsetX(0.1);
      }


      test.runAssertions(getTestData(testName));

      ThreadTools.sleepForever();
      PrintTools.info("Test passed.");
   }
}

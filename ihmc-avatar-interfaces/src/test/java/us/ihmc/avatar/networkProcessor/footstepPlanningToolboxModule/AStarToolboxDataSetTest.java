package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pubsub.DomainFactory;

public class AStarToolboxDataSetTest extends FootstepPlannerToolboxDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   @Override
   @Test
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   @Test
   public void testCorridor()
   {
      DataSet dataSet = DataSetIOTools.loadDataSet("20190219_182005_Corridor");

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, new DefaultFootstepPlanningParameters()
      {
         @Override public boolean checkForBodyBoxCollisions()
         {
            return true;
         }
      });

      runAssertions(dataSet);
   }

   @Test
   public void testBetweenTwoBollards()
   {
      DataSet dataSet = DataSetIOTools.loadDataSet("20190219_182005_Bollards");

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, new DefaultFootstepPlanningParameters()
      {
         @Override public boolean checkForBodyBoxCollisions()
         {
            return true;
         }

         @Override public double getBodyBoxDepth()
         {
            return 0.45;
         }

         @Override public double getBodyBoxWidth()
         {
            return 0.9;
         }

         @Override public double getBodyBoxBaseX()
         {
            return 0.1;
         }
      });

      runAssertions(dataSet);
   }

   public static void main(String[] args) throws Exception
   {
      AStarToolboxDataSetTest test = new AStarToolboxDataSetTest();

      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), "20171218_204917_FlatGround");

      ThreadTools.sleepForever();
      test.tearDown();
      PrintTools.info("Test passed.");
   }
}

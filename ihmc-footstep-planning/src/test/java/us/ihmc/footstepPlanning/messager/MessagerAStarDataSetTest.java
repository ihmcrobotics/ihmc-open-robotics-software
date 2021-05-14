package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSetName;

public class MessagerAStarDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   protected boolean getPlanBodyPath()
   {
      return false;
   }

   @Override
   protected boolean getPerformAStarSearch()
   {
      return true;
   }

   @Override
   protected String getTestNamePrefix()
   {
      return "a_star";
   }

   @Override
   @Test
   public void testDataSets()
   {
      super.testDataSets();
   }

   @Override
   @Test
   @Disabled
   public void testDatasetsInDevelopment()
   {
      super.testDatasetsInDevelopment();
   }

   public static void main(String[] args) throws Exception
   {
//      MessagerAStarDataSetTest messagerAStarDataSetTest = new MessagerAStarDataSetTest();
//      messagerAStarDataSetTest.setup();
//      messagerAStarDataSetTest.testDataSets();

//      20190219_182005_Wall
//      20171215_211034_DoorwayNoCeiling

      MessagerAStarDataSetTest test = new MessagerAStarDataSetTest();
      test.VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20171215_211034_DoorwayNoCeiling);
      test.tearDown();
   }
}

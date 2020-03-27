package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSetName;

@Tag("footstep-planning-slow")
public class MessagerPlanThenSnapDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   protected boolean getPlanBodyPath()
   {
      return false;
   }

   @Override
   protected boolean getPerformAStarSearch()
   {
      return false;
   }

   @Override
   protected String getTestNamePrefix()
   {
      return "plan_then_snap";
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
      MessagerPlanThenSnapDataSetTest test = new MessagerPlanThenSnapDataSetTest();
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20190219_182005_SteppingStones);
      test.tearDown();
   }
}

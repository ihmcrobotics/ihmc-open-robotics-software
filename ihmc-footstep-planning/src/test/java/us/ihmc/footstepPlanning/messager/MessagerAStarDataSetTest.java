package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pathPlanning.DataSetName;

public class MessagerAStarDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
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
      MessagerAStarDataSetTest test = new MessagerAStarDataSetTest();
      test.VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20171218_205120_BodyPathPlannerEnvironment);
      test.tearDown();

   }
}

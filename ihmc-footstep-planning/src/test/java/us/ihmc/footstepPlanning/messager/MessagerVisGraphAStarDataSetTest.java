package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pathPlanning.DataSetName;

public class MessagerVisGraphAStarDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   @Override
   @Test
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   @Override
   @Test
   @Disabled
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      super.testDatasetsWithoutOcclusionInDevelopment();
   }

   public static void main(String[] args) throws Exception
   {
      MessagerVisGraphAStarDataSetTest test = new MessagerVisGraphAStarDataSetTest();
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20171218_205120_BodyPathPlannerEnvironment);
      ThreadTools.sleepForever();
      test.tearDown();
   }
}

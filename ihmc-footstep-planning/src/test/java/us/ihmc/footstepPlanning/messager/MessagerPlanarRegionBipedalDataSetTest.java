package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.log.LogTools;

@Tag("allocation")
public class MessagerPlanarRegionBipedalDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLANAR_REGION_BIPEDAL;
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
//      VISUALIZE = true;
      MessagerPlanarRegionBipedalDataSetTest test = new MessagerPlanarRegionBipedalDataSetTest();
      String prefix = "unitTestDataSets/test/";
      test.setup();
      LogTools.info("Running test.");
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171215_214801_StairsUpDown");
      LogTools.info("Test finished.");
      test.tearDown();

   }
}

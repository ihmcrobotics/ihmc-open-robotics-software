package us.ihmc.footstepPlanning.messager;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.log.LogTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.SLOW)
public class MessagerPlanarRegionBipedalDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLANAR_REGION_BIPEDAL;
   }

   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 122.8)
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 90.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
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

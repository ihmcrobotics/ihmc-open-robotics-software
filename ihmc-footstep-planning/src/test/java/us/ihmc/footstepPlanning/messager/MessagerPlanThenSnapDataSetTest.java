package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@Tag("allocation")
public class MessagerPlanThenSnapDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLAN_THEN_SNAP;
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
      MessagerPlanThenSnapDataSetTest test = new MessagerPlanThenSnapDataSetTest();
      String prefix = "unitTestData/testable/";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_205040_SimpleMaze");
      test.tearDown();

   }
}

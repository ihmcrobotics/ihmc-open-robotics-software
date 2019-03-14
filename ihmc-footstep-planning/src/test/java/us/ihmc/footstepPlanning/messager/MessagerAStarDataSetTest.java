package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;

public class MessagerAStarDataSetTest extends FootstepPlannerDataSetTest
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

   @Override
   @Test
   @Disabled
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      super.testDatasetsWithoutOcclusionInDevelopment();
   }

   public static void main(String[] args) throws Exception
   {
      MessagerAStarDataSetTest test = new MessagerAStarDataSetTest();
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), "20171218_204917_FlatGround");
      test.tearDown();

   }
}

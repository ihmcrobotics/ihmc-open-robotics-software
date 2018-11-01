package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;

import java.util.List;

import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.getTestData;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class MessagerAStarOnRoughTerrainTest extends MessagerFootstepPlannerOnRoughTerrainTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   @ContinuousIntegrationTest(estimatedDuration = 20)
   @Test(timeout = 30000000)
   public void test()
   {
      super.test();
   }
}

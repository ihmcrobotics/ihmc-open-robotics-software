package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;

import java.util.List;

import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.getTestData;

public class MessagerAStarOnRoughTerrainTest extends MessagerFootstepPlannerOnRoughTerrainTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   @Test
   public void test()
   {
      super.test();
   }
}

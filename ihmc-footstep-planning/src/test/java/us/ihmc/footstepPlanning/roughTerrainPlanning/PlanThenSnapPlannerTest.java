package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PlanThenSnapPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private PlanThenSnapPlanner planner;

   protected static boolean visualize = true;
   private static boolean keepUp = false;


   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

   @Override
   public boolean keepUp()
   {
      return keepUp;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 30000)
   public void testSteppingStones()
   {
      super.testSteppingStones();
   }

   @Override
   public void setupInternal()
   {
      planner = new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), PlannerTools.createDefaultFootPolygons());
   }

   @Override
   public void destroyInternal()
   {
      planner = null;
   }
}

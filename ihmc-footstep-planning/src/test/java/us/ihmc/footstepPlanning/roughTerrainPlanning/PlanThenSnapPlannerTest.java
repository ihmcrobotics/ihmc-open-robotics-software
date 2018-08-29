package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.PlannerTools;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PlanThenSnapPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private final PlanThenSnapPlanner planner = new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), PlannerTools.createDefaultFootPolygons());

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

   @ContinuousIntegrationTest(estimatedDuration = 0.2, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 30000)
   public void testSteppingStones()
   {
      super.testSteppingStones();
   }

}

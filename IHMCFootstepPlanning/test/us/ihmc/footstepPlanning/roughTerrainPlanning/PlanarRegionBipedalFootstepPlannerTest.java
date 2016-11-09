package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class PlanarRegionBipedalFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBox()
   {
      super.testSimpleStepOnBox();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRandomEnvironment()
   {
      super.testRandomEnvironment();
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner();

      planner.setMaxStepReach(1.0);

      double idealFootstepLength = 0.2;
      double idealFootstepWidth = 0.3;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      planner.setFeetPolygons(PlanningTestTools.createDefaultFootPolygons());
      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

}

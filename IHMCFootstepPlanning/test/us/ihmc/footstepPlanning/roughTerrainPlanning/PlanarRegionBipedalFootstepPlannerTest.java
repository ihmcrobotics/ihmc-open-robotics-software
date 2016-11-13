package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class PlanarRegionBipedalFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testOnStairCase()
   {
      super.testOnStaircase(new Vector3d(0.1, 0.1, -0.1), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBox()
   {
      super.testSimpleStepOnBox(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBoxTwo()
   {
      super.testSimpleStepOnBoxTwo(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRandomEnvironment()
   {
      super.testRandomEnvironment(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleGaps()
   {
      super.testSimpleGaps(true);
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner();

      planner.setMaximumStepReach(0.45);
      planner.setMaximumStepZ(0.25);
      planner.setMaximumStepYaw(0.15);
      planner.setMinimumStepWidth(0.15);
      planner.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (visualize)
      {
         SCSPlanarRegionBipedalFootstepPlannerVisualizer visualizer = new SCSPlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame);
         planner.setBipedalFootstepPlannerListener(visualizer);
      }

      planner.setMaximumNumberOfNodesToExpand(100);

      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

}

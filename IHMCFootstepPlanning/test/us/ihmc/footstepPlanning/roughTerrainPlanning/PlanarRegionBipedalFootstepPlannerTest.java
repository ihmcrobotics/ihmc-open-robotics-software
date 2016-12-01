package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PlanarRegionBipedalFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testOnStairCase()
   {
      super.testOnStaircase(new Vector3d(), true);
   }

   @Override
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

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRandomEnvironment()
   {
      super.testRandomEnvironment(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleGaps()
   {
      super.testSimpleGaps(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testOverCinderBlockField()
   {
      super.testOverCinderBlockField(true);
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner(registry);

      planner.setMaximumStepReach(0.55); //0.45);
      planner.setMaximumStepZ(0.25);
      planner.setMaximumStepXWhenForwardAndDown(0.25);
      planner.setMaximumStepZWhenForwardAndDown(0.25);
      planner.setMaximumStepYaw(0.15);
      planner.setMinimumStepWidth(0.15);
      planner.setMinimumFootholdPercent(0.8);

      planner.setWiggleInsideDelta(0.05);
      planner.setMaximumXYWiggleDistance(1.0);
      planner.setMaximumYawWiggle(0.1);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (visualize)
      {
         PlanarRegionBipedalFootstepPlannerVisualizer visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0,
                                                                                                                                                       footPolygonsInSoleFrame);
         planner.setBipedalFootstepPlannerListener(visualizer);
         visualizer.getYoVariableRegistry().addChild(registry);
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

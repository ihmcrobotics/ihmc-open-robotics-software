package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.flatGroundPlanning.FootstepPlannerOnFlatGroundTest;
import us.ihmc.footstepPlanning.graphSearch.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class DepthFirstFootstepPlannerOnFlatTest extends FootstepPlannerOnFlatGroundTest
{
   private YoVariableRegistry registry;
   private YoFootstepPlannerParameters parameters;
   private DepthFirstFootstepPlanner planner;

   private static final boolean visualize = false; // !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean showPlannerVisualizer = false;

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
      planner.setMaximumNumberOfNodesToExpand(10000);
      planner.setTimeout(10.0);
      super.testJustStraightLine(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testATightTurn()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testATightTurn(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testStraightLineWithInitialTurn()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(2.0);
      planner.setExitAfterInitialSolution(false);
      super.testStraightLineWithInitialTurn(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustTurnInPlace()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(2.0);
      planner.setExitAfterInitialSolution(false);
      super.testJustTurnInPlace(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = 300000)
   public void testRandomPoses()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(2.0);
      planner.setExitAfterInitialSolution(false);
      super.testRandomPoses(true);
   }

   @Before
   public void setupPlanner()
   {
      registry = new YoVariableRegistry("test");
      parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters());
      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      PlanarRegionBipedalFootstepPlannerVisualizer visualizer = null;
      if (showPlannerVisualizer)
      {
         visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0, footPolygonsInSoleFrame, registry);
      }

      AlwaysValidNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      ConstantFootstepCost footstepCost = new ConstantFootstepCost(1.0);
      planner = new DepthFirstFootstepPlanner(parameters, snapper, nodeChecker, footstepCost, registry);
      planner.setFeetPolygons(footPolygonsInSoleFrame);
      planner.setMaximumNumberOfNodesToExpand(1000);
      planner.setBipedalFootstepPlannerListener(visualizer);
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
}

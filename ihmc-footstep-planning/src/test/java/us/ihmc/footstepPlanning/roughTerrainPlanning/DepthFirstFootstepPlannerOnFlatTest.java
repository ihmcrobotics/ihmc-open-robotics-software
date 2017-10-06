package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.flatGroundPlanning.FootstepPlannerOnFlatGroundTest;
import us.ihmc.footstepPlanning.graphSearch.*;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class DepthFirstFootstepPlannerOnFlatTest extends FootstepPlannerOnFlatGroundTest
{
   private YoVariableRegistry registry;
   private BipedalFootstepPlannerParameters parameters;
   private DepthFirstFootstepPlanner planner;

   private static final boolean visualize = false; // !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean showPlannerVisualizer = false;

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
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
      parameters = new BipedalFootstepPlannerParameters(registry);

      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper(footPolygonsInSoleFrame);
      AlwaysValidNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      ConstantFootstepCost footstepCost = new ConstantFootstepCost(1.0);
      planner = new DepthFirstFootstepPlanner(parameters, snapper, nodeChecker, footstepCost, registry);

      setDefaultParameters();
   }

   private void setDefaultParameters()
   {
      parameters.setMaximumStepReach(0.4);
      parameters.setMaximumStepZ(0.25);
      parameters.setMaximumStepXWhenForwardAndDown(0.25);
      parameters.setMaximumStepZWhenForwardAndDown(0.25);
      parameters.setMaximumStepYaw(0.15);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (showPlannerVisualizer)
      {
         PlanarRegionBipedalFootstepPlannerVisualizer visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0,
                                                                                                                                                       footPolygonsInSoleFrame, registry);
         planner.setBipedalFootstepPlannerListener(visualizer);
      }

      planner.setMaximumNumberOfNodesToExpand(1000);
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

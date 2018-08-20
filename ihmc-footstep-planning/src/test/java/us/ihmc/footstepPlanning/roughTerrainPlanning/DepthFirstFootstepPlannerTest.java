package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class DepthFirstFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private YoVariableRegistry registry;
   private YoFootstepPlannerParameters parameters;
   private DepthFirstFootstepPlanner planner;

   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean showPlannerVisualizer = false;

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testOnStaircase()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testOnStaircase();

      if (showPlannerVisualizer)
         ThreadTools.sleepForever();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testSimpleStepOnBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleStepOnBox();
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testSimpleStepOnBoxTwo()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleStepOnBoxTwo();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testRandomEnvironment()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testRandomEnvironment();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.2)
   @Test(timeout = 51000)
   public void testSimpleGaps()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleGaps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.1)
   @Test(timeout = 50000)
   public void testOverCinderBlockField()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testOverCinderBlockField();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStepAfterPitchedUp()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testStepAfterPitchedUp();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStepAfterPitchedDown()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testStepAfterPitchedDown();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testCompareStepBeforeGap()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testCompareStepBeforeGap();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 75000)
   public void testWalkingAroundBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testWalkingAroundBox();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testSteppingStones()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testSteppingStones();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.2, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 51000)
   public void testPartialGaps()
   {
      super.testPartialGaps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.2, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 51000)
   public void testSpiralStaircase()
   {
      super.testSpiralStaircase();
   }

   @Before
   public void setupPlanner()
   {
      registry = new YoVariableRegistry("test");
      parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters());
      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();

      PlanarRegionBipedalFootstepPlannerVisualizer visualizer = null;
      if (showPlannerVisualizer)
         visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0, footPolygonsInSoleFrame, registry);

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, parameters, visualizer);
      SnapBasedNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygonsInSoleFrame, snapper);
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      if(showPlannerVisualizer)
      {
         visualizer.setFootstepSnapper(snapper);
         nodeChecker.addPlannerListener(visualizer);
      }

      planner = new DepthFirstFootstepPlanner(parameters, snapper, nodeChecker, stepCostCalculator, registry);
      planner.setFeetPolygons(footPolygonsInSoleFrame);
      planner.setMaximumNumberOfNodesToExpand(100);
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

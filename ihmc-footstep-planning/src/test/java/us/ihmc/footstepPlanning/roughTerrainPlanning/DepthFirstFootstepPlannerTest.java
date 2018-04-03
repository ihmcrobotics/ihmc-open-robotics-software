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

   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testOnStairCase()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testOnStaircase(new Vector3D(), true);

      if (showPlannerVisualizer)
         ThreadTools.sleepForever();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleStepOnBox(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBoxTwo()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleStepOnBoxTwo(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testRandomEnvironment()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testRandomEnvironment(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testSimpleGaps()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleGaps(true);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testOverCinderBlockField()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testOverCinderBlockField(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testStepAfterPitchedUp()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testCompareAfterPitchedStep(!visualize, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testStepAfterPitchedDown()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testCompareAfterPitchedStep(!visualize, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testStepBeforeGap()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testCompareStepBeforeGap(!visualize);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testWalkingAroundBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testWalkingAroundBox();
   }

   public void testSteppingStones()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testSteppingStones(!visualize);
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

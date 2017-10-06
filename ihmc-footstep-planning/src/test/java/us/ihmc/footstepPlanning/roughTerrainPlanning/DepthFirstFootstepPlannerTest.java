package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.*;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapAndWiggleBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class DepthFirstFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private YoVariableRegistry registry;
   private BipedalFootstepPlannerParameters parameters;
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

      if(showPlannerVisualizer)
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

   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testPartialGaps()
   {
      parameters.setMinimumFootholdPercent(0.4);
      parameters.setRejectIfCannotFullyWiggleInside(false);

      super.testPartialGaps(!visualize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10000.0)
   @Test(timeout = 300000)
   public void testWalkingAroundBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testWalkingAroundBox();
   }

   @Before
   public void setupPlanner()
   {
      registry = new YoVariableRegistry("test");
      parameters = new BipedalFootstepPlannerParameters(registry);
      setDefaultParameters();
      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(parameters);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(parameters, null);
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      snapper.setFootPolygonsInSoleFrame(footPolygonsInSoleFrame);
      nodeChecker.setFeetPolygons(footPolygonsInSoleFrame);

      planner = new DepthFirstFootstepPlanner(parameters, snapper, nodeChecker, stepCostCalculator, registry);
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (showPlannerVisualizer)
      {
         PlanarRegionBipedalFootstepPlannerVisualizer visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0, footPolygonsInSoleFrame, registry);
         planner.setBipedalFootstepPlannerListener(visualizer);
      }

      planner.setMaximumNumberOfNodesToExpand(100);
   }

   private void setDefaultParameters()
   {
      parameters.setMaximumStepReach(0.55); //0.45);
      parameters.setMaximumStepZ(0.25);
      parameters.setMaximumStepXWhenForwardAndDown(0.25);
      parameters.setMaximumStepZWhenForwardAndDown(0.25);
      parameters.setMaximumStepYaw(0.15);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.95);

      parameters.setMinimumStepLength(-0.03);

      parameters.setWiggleInsideDelta(0.05);
      parameters.setMaximumXYWiggleDistance(1.0);
      parameters.setMaximumYawWiggle(0.1);

      parameters.setCliffHeightToShiftAwayFrom(0.04);
      parameters.setMinimumDistanceFromCliffBottoms(0.22);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);
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

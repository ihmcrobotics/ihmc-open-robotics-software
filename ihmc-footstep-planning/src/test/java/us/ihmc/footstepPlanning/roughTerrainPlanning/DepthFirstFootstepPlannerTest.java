package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DepthFirstFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private YoFootstepPlannerParameters parameters;
   private DepthFirstFootstepPlanner planner;

   private static final boolean showPlannerVisualizer = false;

   private static boolean keepUp = false;

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   @Test
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
   @Test
   public void testSimpleStepOnBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleStepOnBox();
   }

   @Test
   public void testSimpleStepOnBoxTwo()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleStepOnBoxTwo();
   }

   @Override
   @Test
   public void testRandomEnvironment()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testRandomEnvironment();
   }

   @Override
   @Test
   public void testSimpleGaps()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testSimpleGaps();
   }

   @Override
   @Test
   public void testOverCinderBlockField()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testOverCinderBlockField();
   }

   @Override
   @Test
   public void testStepAfterPitchedUp()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testStepAfterPitchedUp();
   }

   @Override
   @Test
   public void testStepAfterPitchedDown()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testStepAfterPitchedDown();
   }

   @Override
   @Test
   public void testCompareStepBeforeGap()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testCompareStepBeforeGap();
   }

   @Override
   @Test
   public void testWalkingAroundBox()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testWalkingAroundBox();
   }

   @Override
   @Test
   public void testSteppingStones()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setExitAfterInitialSolution(false);
      planner.setTimeout(15.0);
      super.testSteppingStones();
   }

   @Override
   @Disabled
   @Test
   public void testPartialGaps()
   {
      super.testPartialGaps();
   }

   @Override
   @Disabled
   @Test
   public void testSpiralStaircase()
   {
      super.testSpiralStaircase();
   }

   @Override
   public void setupInternal()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters());
      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlannerTools.createDefaultFootPolygons();

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, parameters);
      SnapBasedNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygonsInSoleFrame, snapper);
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      planner = new DepthFirstFootstepPlanner(parameters, snapper, nodeChecker, stepCostCalculator, registry);
      planner.setFeetPolygons(footPolygonsInSoleFrame);
      planner.setMaximumNumberOfNodesToExpand(100);
   }

   @Override
   public void destroyInternal()
   {
      planner = null;
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
}

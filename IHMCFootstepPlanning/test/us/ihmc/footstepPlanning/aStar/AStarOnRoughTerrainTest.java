package us.ihmc.footstepPlanning.aStar;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.aStar.implementations.EuclidianBasedCost;
import us.ihmc.footstepPlanning.aStar.implementations.EuclidianDistanceHeuristics;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleNodeChecker;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleSideBasedExpansion;
import us.ihmc.footstepPlanning.roughTerrainPlanning.FootstepPlannerOnRoughTerrainTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarOnRoughTerrainTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean visualizePlanner = false;
   private AStarFootstepPlanner planner;
   private FootstepNodeVisualization visualization = null;

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testWalkingAroundBox()
   {
      super.testWalkingAroundBox();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testWalkingAroundHole()
   {
      super.testWalkingAroundHole();
   }

   @Before
   public void createPlanner()
   {
      // create planner
      SimpleNodeChecker nodeChecker = new SimpleNodeChecker();
      EuclidianDistanceHeuristics heuristics = new EuclidianDistanceHeuristics();
      SimpleSideBasedExpansion expansion = new SimpleSideBasedExpansion();
      EuclidianBasedCost stepCostCalculator = new EuclidianBasedCost();

      if (visualizePlanner)
         visualization = new FootstepNodeVisualization(2000, 1.0, null);
      planner = new AStarFootstepPlanner(nodeChecker, heuristics, expansion, stepCostCalculator, visualization);
   }

   @After
   public void destroyPlanner()
   {
      planner = null;

      if (visualizePlanner)
      {
         for (int i = 0; i < 1000; i++)
            visualization.tickAndUpdate();
         visualization.showAndSleep(true);
      }
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      if (visualizePlanner)
         return false;
      return visualize;
   }
}

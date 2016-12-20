package us.ihmc.footstepPlanning.aStar;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.aStar.implementations.EuclidianBasedCost;
import us.ihmc.footstepPlanning.aStar.implementations.EuclidianDistanceHeuristics;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleNodeChecker;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleSideBasedExpansion;
import us.ihmc.footstepPlanning.flatGroundPlanning.FootstepPlannerOnFlatGroundTest;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarOnFlatTest extends FootstepPlannerOnFlatGroundTest
{
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean visualizePlanner = false;
   private AStarFootstepPlanner planner;
   private FootstepNodeVisualization visualization = null;

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
      super.testJustStraightLine(true);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testATightTurn()
   {
      super.testATightTurn(true);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testStraightLineWithInitialTurn()
   {
      super.testStraightLineWithInitialTurn(true);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustTurnInPlace()
   {
      super.testJustTurnInPlace(true);
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
         visualization = new FootstepNodeVisualization(1000, 1.0, null);
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
      return visualize;
   }

}

package us.ihmc.footstepPlanning.graphSearch.aStar;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.flatGroundPlanning.FootstepPlannerOnFlatGroundTest;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.SimpleSideBasedExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarOnFlatTest extends FootstepPlannerOnFlatGroundTest
{
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean visualizePlanner = false;
   private AStarFootstepPlanner planner;
   private FootstepNodeVisualization visualization = null;

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJustStraightLine()
   {
      super.testJustStraightLine(true);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testATightTurn()
   {
      super.testATightTurn(true);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testStraightLineWithInitialTurn()
   {
      super.testStraightLineWithInitialTurn(true);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 2.8)
   @Test(timeout = 30000)
   public void testJustTurnInPlace()
   {
      super.testJustTurnInPlace(true);
   }

   @Before
   public void createPlanner()
   {
      if (visualizePlanner)
         visualization = new FootstepNodeVisualization(1000, 1.0, null);
      SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createDefaultFootPolygons();
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      SimpleSideBasedExpansion expansion = new SimpleSideBasedExpansion(parameters);
      planner = AStarFootstepPlanner.createFlatGroundPlanner(parameters, visualization, footPolygons, expansion, new YoVariableRegistry("TestRegistry"));
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

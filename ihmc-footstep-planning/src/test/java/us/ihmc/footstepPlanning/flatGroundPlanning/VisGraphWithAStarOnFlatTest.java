package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.aStar.FootstepNodeVisualization;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.SimpleSideBasedExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.VisibilityGraphWithAStarPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisGraphWithAStarOnFlatTest extends FootstepPlannerOnFlatGroundTest
{
   private static final boolean visualize = false;//!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean keepUp = false;
   private static final boolean visualizePlanner = false;
   private FootstepPlanner planner;
   private FootstepNodeVisualization visualization = null;

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @BeforeEach
   public void createPlanner()
   {
      if (visualizePlanner)
         visualization = new FootstepNodeVisualization(1000, 1.0, null);
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      VisibilityGraphsParametersBasics visibilityGraphsParameters = new DefaultVisibilityGraphParameters();
      planner = new VisibilityGraphWithAStarPlanner(parameters, visibilityGraphsParameters, footPolygons,
                                                    null, new YoVariableRegistry("TestRegistry"));
   }

   @AfterEach
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

   @Override
   public boolean keepUp()
   {
      return keepUp;
   }
}

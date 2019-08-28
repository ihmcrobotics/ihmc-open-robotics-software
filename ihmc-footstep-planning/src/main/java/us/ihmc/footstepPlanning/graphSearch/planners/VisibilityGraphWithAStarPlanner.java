package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.VisibilityGraphPathPlanner;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisibilityGraphWithAStarPlanner extends BodyPathAndFootstepPlannerWrapper
{
   private static final String prefix = "VisGraph";

   public VisibilityGraphWithAStarPlanner(FootstepPlannerParametersReadOnly parameters, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                                          SideDependentList<ConvexPolygon2D> footPolygons, YoGraphicsListRegistry graphicsListRegistry,
                                          YoVariableRegistry parentRegistry, BipedalFootstepPlannerListener... listeners)
   {
      super(prefix, parameters, parentRegistry, graphicsListRegistry);

      waypointPathPlanner = new VisibilityGraphPathPlanner(parameters, visibilityGraphsParameters, parentRegistry);
      footstepPlanner = new BodyPathBasedAStarPlanner(prefix, bodyPathPlanner, parameters, footPolygons,
                                                      parameters.getAStarHeuristicsWeight(), registry, listeners);
   }
}

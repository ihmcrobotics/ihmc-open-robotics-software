package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.VisibilityGraphPathPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisibilityGraphWithAStarPlanner extends BodyPathAndFootstepPlannerWrapper
{
   private static final String prefix = "VisGraph";

   public VisibilityGraphWithAStarPlanner(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettingsReadOnly,
                                          VisibilityGraphsParameters visibilityGraphsParameters, YoGraphicsListRegistry graphicsListRegistry,
                                          YoVariableRegistry parentRegistry)
   {
      super(prefix, parameters, parentRegistry, graphicsListRegistry);

      waypointPathPlanner = new VisibilityGraphPathPlanner(prefix, visibilityGraphsParameters, parentRegistry);
      footstepPlanner = new QuadrupedBodyPathBasedAStarPlanner(prefix, bodyPathPlanner, parameters, xGaitSettingsReadOnly, registry);
   }
}

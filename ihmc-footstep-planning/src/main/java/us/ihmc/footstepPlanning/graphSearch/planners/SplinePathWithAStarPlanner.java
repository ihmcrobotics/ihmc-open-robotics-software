package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.SplinePathPlanner;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SplinePathWithAStarPlanner extends BodyPathAndFootstepPlannerWrapper
{
   private static final String prefix = "splineBased";

   public SplinePathWithAStarPlanner(FootstepPlannerParametersReadOnly parameters, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry parentRegistry,
                                     YoGraphicsListRegistry graphicsListRegistry)
   {
      super(prefix, parameters, parentRegistry, graphicsListRegistry);

      waypointPathPlanner = new SplinePathPlanner(parameters, registry);
      footstepPlanner = new BodyPathBasedAStarPlanner(prefix, bodyPathPlanner, parameters, footPolygons,
                                                      parameters.getBodyPathBasedHeuristicsWeight(), registry);
   }
}

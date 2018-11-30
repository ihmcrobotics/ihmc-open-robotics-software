package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.SplinePathPlanner;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class SplinePathWithAStarPlanner extends BodyPathAndFootstepPlannerWrapper
{
   private static final String prefix = "splineBased";

   public SplinePathWithAStarPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry parentRegistry,
                                     YoGraphicsListRegistry graphicsListRegistry)
   {
      super(prefix, parameters, parentRegistry, graphicsListRegistry);

      waypointPathPlanner = new SplinePathPlanner(parameters, registry);
      footstepPlanner = new BodyPathBasedAStarPlanner(prefix, bodyPathPlanner, parameters, footPolygons,
                                                      parameters.getCostParameters().getBodyPathBasedHeuristicsWeight(), registry);
   }
}

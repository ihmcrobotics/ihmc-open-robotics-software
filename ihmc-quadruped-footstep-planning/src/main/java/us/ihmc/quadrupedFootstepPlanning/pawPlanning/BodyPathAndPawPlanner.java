package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawPlanner;

public interface BodyPathAndPawPlanner extends PawPlanner
{
   WaypointsForPawPlanner getWaypointPathPlanner();
   PawPlanner getPawPlanner();

   PawPlanningResult planPath();
   BodyPathPlan getPathPlan();
}

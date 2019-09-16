package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawStepPlanner;

public interface BodyPathAndPawPlanner extends PawStepPlanner
{
   WaypointsForPawStepPlanner getWaypointPathPlanner();
   PawStepPlanner getPawStepPlanner();

   PawStepPlanningResult planPath();
   BodyPathPlan getPathPlan();
}

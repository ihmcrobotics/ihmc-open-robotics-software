package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForQuadrupedFootstepPlanner;

public interface QuadrupedBodyPathAndFootstepPlanner extends QuadrupedFootstepPlanner
{
   WaypointsForQuadrupedFootstepPlanner getWaypointPathPlanner();
   QuadrupedFootstepPlanner getFootstepPlanner();

   FootstepPlanningResult planPath();
   BodyPathPlan getPathPlan();
}

package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;

public interface QuadrupedBodyPathAndFootstepPlanner extends QuadrupedFootstepPlanner
{

   FootstepPlanningResult planPath();
   BodyPathPlan getPathPlan();
}

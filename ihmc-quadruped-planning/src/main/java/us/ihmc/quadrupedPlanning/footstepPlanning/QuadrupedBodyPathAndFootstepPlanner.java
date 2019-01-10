package us.ihmc.quadrupedPlanning.footstepPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedPlanning.footstepPlanning.QuadrupedFootstepPlanner;

public interface QuadrupedBodyPathAndFootstepPlanner extends QuadrupedFootstepPlanner
{
   void planPath();

   BodyPathPlan getPathPlan();
}

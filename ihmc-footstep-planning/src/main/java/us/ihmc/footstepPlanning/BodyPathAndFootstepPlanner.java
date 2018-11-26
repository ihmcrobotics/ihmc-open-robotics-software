package us.ihmc.footstepPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;

public interface BodyPathAndFootstepPlanner extends FootstepPlanner
{
   /**
    * This plans the body path, if available.
    */
   default FootstepPlanningResult planPath()
   {
      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   default BodyPathPlan getPathPlan()
   {
      return null;
   }
}

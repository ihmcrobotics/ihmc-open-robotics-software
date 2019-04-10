package us.ihmc.footstepPlanning;

import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;

public interface AnytimeFootstepPlanner extends FootstepPlanner
{
   /**
    * Returns the current best plan. The metric for "best" is determined by the implementing class
    */
   public FootstepPlan getBestPlanYet();

   /**
    * Resets the start of the search to this footstep
    * @param footstep step to be executed
    */
   public void executingFootstep(SimpleFootstep footstep);

   public boolean isBestPlanYetOptimal();
}

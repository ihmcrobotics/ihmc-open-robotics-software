package us.ihmc.footstepPlanning.graphSearch.listeners;

public interface NodeFailureEventListener extends BipedalFootstepPlannerListener
{
   /**
    * Sets the heuristic search policy to be performed in the event of the intended node failure.
    */
   void setHeuristicSearchPolicy(PlannerHeuristicNodeSearchPolicy heuristicSearchPolicy);
}

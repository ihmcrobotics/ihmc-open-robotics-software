package us.ihmc.footstepPlanning;

import us.ihmc.footstepPlanning.aStar.implementations.ReachableFootstepsBasedExpansion;

public interface FootstepPlannerParameters
{
   public ReachableFootstepsBasedExpansion getReachableFootstepExpansion();
   
   public double getTimeout();
}

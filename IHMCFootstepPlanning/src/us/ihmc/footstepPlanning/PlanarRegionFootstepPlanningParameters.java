package us.ihmc.footstepPlanning;

import us.ihmc.footstepPlanning.aStar.implementations.ReachableFootstepsBasedExpansion;

public interface PlanarRegionFootstepPlanningParameters
{
   public ReachableFootstepsBasedExpansion getReachableFootstepExpansion();
   
   public double getTimeout();
}

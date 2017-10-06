package us.ihmc.footstepPlanning;

import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ReachableFootstepsBasedExpansion;

public interface PlanarRegionFootstepPlanningParameters
{
   public ReachableFootstepsBasedExpansion getReachableFootstepExpansion();
   
   public double getTimeout();
}

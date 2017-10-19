package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.PlanarRegionFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ReachableFootstepsBasedExpansion;

public class AtlasPlanarRegionFootstepPlannerParameters implements PlanarRegionFootstepPlanningParameters
{
   private final double timeout = Double.POSITIVE_INFINITY;

   @Override
   public ReachableFootstepsBasedExpansion getReachableFootstepExpansion()
   {
      return new AtlasReachableFootstepExpansion();
   }

   @Override
   public double getTimeout()
   {
      return timeout;
   }
}

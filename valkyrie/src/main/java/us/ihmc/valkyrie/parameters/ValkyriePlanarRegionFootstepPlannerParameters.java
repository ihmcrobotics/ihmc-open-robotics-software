package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.PlanarRegionFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ReachableFootstepsBasedExpansion;

public class ValkyriePlanarRegionFootstepPlannerParameters implements PlanarRegionFootstepPlanningParameters
{
   private final double timeout = Double.POSITIVE_INFINITY;

   @Override
   public ReachableFootstepsBasedExpansion getReachableFootstepExpansion()
   {
      return new ValkyrieReachableFootstepExpansion();
   }

   @Override
   public double getTimeout()
   {
      return timeout;
   }

}

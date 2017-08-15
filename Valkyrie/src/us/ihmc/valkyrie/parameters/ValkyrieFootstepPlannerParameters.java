package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.PlanarRegionFootstepPlannerParameters;
import us.ihmc.footstepPlanning.aStar.implementations.ReachableFootstepsBasedExpansion;

public class ValkyrieFootstepPlannerParameters implements PlanarRegionFootstepPlannerParameters
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

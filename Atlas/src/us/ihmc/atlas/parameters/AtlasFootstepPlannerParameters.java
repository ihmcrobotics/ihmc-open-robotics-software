package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.aStar.implementations.ReachableFootstepsBasedExpansion;

public class AtlasFootstepPlannerParameters implements FootstepPlannerParameters
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

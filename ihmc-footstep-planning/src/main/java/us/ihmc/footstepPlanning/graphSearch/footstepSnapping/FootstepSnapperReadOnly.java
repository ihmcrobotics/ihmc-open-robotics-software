package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;

public interface FootstepSnapperReadOnly
{
   /**
    * Computes projection of planar step onto the environment.
    *
    * @param footstep step to be projected
    * @param stanceStep stance step, used to determine the maximum region height to consider. if null will snap to highest region
    * @param computeWiggleTransform whether to wiggle step into region
    */
   FootstepSnapDataReadOnly snapFootstep(DiscreteFootstep footstep, DiscreteFootstep stanceStep, boolean computeWiggleTransform);

   /**
    * Projects step to the highest available region and does not compute wiggle transform
    */
   default FootstepSnapDataReadOnly snapFootstep(DiscreteFootstep footstep)
   {
      return snapFootstep(footstep, null, false);
   }
}

package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepNodeSnapperReadOnly
{
   /**
    * Computes projection of planar step onto the environment.
    *
    * @param node step to be projected
    * @param stanceNode stance step, used to determine the maximum region height to consider. if null will snap to highest region
    * @param computeWiggleTransform whether to wiggle step into region
    */
   FootstepNodeSnapDataReadOnly snapFootstepNode(FootstepNode node, FootstepNode stanceNode, boolean computeWiggleTransform);

   /**
    * Projects step to the highest available region and does not compute wiggle transform
    */
   default FootstepNodeSnapDataReadOnly snapFootstepNode(FootstepNode node)
   {
      return snapFootstepNode(node, null, false);
   }
}

package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepNodeSnapperReadOnly
{
   FootstepNodeSnapDataReadOnly snapFootstepNode(FootstepNode node, boolean computeWiggleTransform);

   default FootstepNodeSnapDataReadOnly snapFootstepNode(FootstepNode node)
   {
      return snapFootstepNode(node, false);
   }
}

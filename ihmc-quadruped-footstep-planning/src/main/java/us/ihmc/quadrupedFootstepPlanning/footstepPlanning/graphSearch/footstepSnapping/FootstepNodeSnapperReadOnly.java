package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepNodeSnapperReadOnly
{
   /**
    * Returns snap data if the snapper has a cache of this node's snap, otherwise returns null
    */
   FootstepNodeSnapData getSnapData(FootstepNode node);
}

package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepNodeSnapperReadOnly
{
   /**
    * Returns snap data if the snapper has a cache of this node's snap, otherwise returns null
    * @param node
    * @return
    */
   public FootstepNodeSnapData getSnapData(FootstepNode node);
}

package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.HashSet;

public interface FootstepNodeExpansion
{
   public HashSet<FootstepNode> expandNode(FootstepNode node);
}

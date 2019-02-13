package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.HashSet;

public interface FootstepNodeExpansion
{
   HashSet<FootstepNode> expandNode(FootstepNode node);
}

package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

import java.util.HashSet;

public interface PawNodeExpansion
{
   HashSet<PawNode> expandNode(PawNode node);
}

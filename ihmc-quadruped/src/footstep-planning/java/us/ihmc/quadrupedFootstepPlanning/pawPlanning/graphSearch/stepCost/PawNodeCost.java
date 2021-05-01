package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

public interface PawNodeCost
{
   double compute(PawNode startNode, PawNode endNode);
}

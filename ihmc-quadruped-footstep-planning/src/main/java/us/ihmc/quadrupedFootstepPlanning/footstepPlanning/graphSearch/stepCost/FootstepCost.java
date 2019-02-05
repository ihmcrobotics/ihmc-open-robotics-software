package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepCost
{
   double compute(FootstepNode startNode, FootstepNode endNode);
}

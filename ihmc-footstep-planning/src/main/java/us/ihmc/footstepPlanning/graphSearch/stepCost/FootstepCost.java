package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepCost
{
   public double compute(FootstepNode startNode, FootstepNode endNode);
}

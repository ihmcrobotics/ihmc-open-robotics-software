package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepPlannerHeuristicActionPolicy
{
   void performActionFromNewNode(FootstepNode newNode, FootstepNode previousNode);
}

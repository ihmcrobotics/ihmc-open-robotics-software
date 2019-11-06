package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

import java.util.Collection;
import java.util.List;

public interface BipedalFootstepPlannerListener
{
   void addNode(FootstepNode node, FootstepNode previousNode);

   void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason);

   void plannerFinished(List<FootstepNode> plan, Collection<FootstepNode> expandedNodes, FootstepGraph footstepGraph);

   void reportLowestCostNodeList(List<FootstepNode> plan);

   void tickAndUpdate();
}

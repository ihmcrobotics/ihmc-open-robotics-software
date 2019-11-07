package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeData;

import java.util.HashMap;
import java.util.List;

public interface BipedalFootstepPlannerListener
{
   void addNode(FootstepNode node, FootstepNode previousNode);

   void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason);

   void plannerFinished(List<FootstepNode> plan);

   void reportLowestCostNodeList(List<FootstepNode> plan);

   void tickAndUpdate();

   default HashMap<FootstepNode, List<PlannerNodeData>> getRejectedNodeData()
   {
      return null;
   }
}

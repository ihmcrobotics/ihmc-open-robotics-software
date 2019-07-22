package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners;


import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.List;

public interface QuadrupedFootstepPlannerListener
{
   void addNode(FootstepNode node, FootstepNode previousNode);

   void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason reason);

   void rejectNode(FootstepNode rejectedNode, QuadrupedFootstepPlannerNodeRejectionReason reason);

   void plannerFinished(List<FootstepNode> plan);

   void reportLowestCostNodeList(List<FootstepNode> plan);

   void tickAndUpdate();
}

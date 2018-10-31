package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.List;

public interface BipedalFootstepPlannerListener
{
   void addNode(FootstepNode node, FootstepNode previousNode);

   void rejectNode(FootstepNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason);

   void planWasFound(List<FootstepNode> plan);

   void tickAndUpdate();
}

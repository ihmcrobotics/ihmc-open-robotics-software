package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface BipedalFootstepPlannerListener
{
   void addNode(FootstepNode node);

   void rejectNode(FootstepNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason);

   void tickAndUpdate();
}

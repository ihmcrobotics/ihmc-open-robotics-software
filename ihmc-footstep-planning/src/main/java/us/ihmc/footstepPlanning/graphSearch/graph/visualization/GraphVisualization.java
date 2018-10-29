package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface GraphVisualization
{
   void addNode(FootstepNode node);

   void rejectNode(FootstepNode node, BipedalFootstepPlannerNodeRejectionReason reason);

   void tickAndUpdate();
}

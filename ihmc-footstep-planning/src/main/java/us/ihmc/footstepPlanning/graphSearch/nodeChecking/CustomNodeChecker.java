package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

public interface CustomNodeChecker
{
   boolean isNodeValid(FootstepNode candidateNode, FootstepNode stanceNode);

   BipedalFootstepPlannerNodeRejectionReason getRejectionReason();
}

package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface FootstepPlannerHeuristicSearchPolicy
{
   void attachActionPolicy(FootstepPlannerHeuristicActionPolicy actionPolicy);
   void performActionPoliciesForNewNode();
   boolean performSearchForNewNode(FootstepNode rejectedNode, FootstepNode parentNode);
   FootstepNode pollNewValidNode();
}

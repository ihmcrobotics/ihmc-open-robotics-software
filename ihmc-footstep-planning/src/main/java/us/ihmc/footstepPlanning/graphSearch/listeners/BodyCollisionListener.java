package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

import java.util.List;

public class BodyCollisionListener implements BipedalFootstepPlannerListener
{
   public FootstepPlannerHeuristicSearchPolicy heuristicSearchPolicy = null;


   public void setHeuristicSearchPolicy(FootstepPlannerHeuristicSearchPolicy heuristicSearchPolicy)
   {
      this.heuristicSearchPolicy = heuristicSearchPolicy;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {

   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (reason.equals(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY) && heuristicSearchPolicy != null)
      {
         boolean foundNewNode = heuristicSearchPolicy.performSearchForNewNode(rejectedNode, parentNode);
         if (foundNewNode)
         {
            heuristicSearchPolicy.performActionPoliciesForNewNode();
         }
      }

   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
   }

   @Override
   public void tickAndUpdate()
   {
   }
}

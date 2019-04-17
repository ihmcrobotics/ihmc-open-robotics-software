package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class BodyCollisionListener implements NodeFailureEventListener
{
   private static final int maximumActionPolicyExecutions = 1;

   public PlannerHeuristicNodeSearchPolicy heuristicSearchPolicy = null;
   private final AtomicInteger policyExecutionCounter = new AtomicInteger();

   public void setHeuristicSearchPolicy(PlannerHeuristicNodeSearchPolicy heuristicSearchPolicy)
   {
      this.heuristicSearchPolicy = heuristicSearchPolicy;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if(previousNode == null)
      {
         policyExecutionCounter.set(0);
      }
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (reason.equals(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY) && heuristicSearchPolicy != null && policyExecutionCounter.get() < maximumActionPolicyExecutions)
      {
         boolean foundNewNode = heuristicSearchPolicy.performSearchForValidNode(rejectedNode, parentNode);
         if (foundNewNode)
         {
            heuristicSearchPolicy.executeActionPoliciesForNewValidNode();
            policyExecutionCounter.incrementAndGet();
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

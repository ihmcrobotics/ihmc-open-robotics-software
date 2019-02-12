package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization;

import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AStarMessagerListener implements QuadrupedFootstepPlannerListener
{
   private final Messager messager;

   private final HashMap<FootstepNode, List<FootstepNode>> validNodesThisTick = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> invalidNodesThisTick = new HashMap<>();
   private final HashMap<QuadrupedFootstepPlannerNodeRejectionReason, List<FootstepNode>> nodesRejectedThisTick = new HashMap<>();

   public AStarMessagerListener(Messager messager)
   {
      this.messager = messager;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (!validNodesThisTick.containsKey(node))
         validNodesThisTick.put(node, new ArrayList<>());

      validNodesThisTick.get(node).add(previousNode);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason reason)
   {
      if (!validNodesThisTick.containsKey(parentNode))
         validNodesThisTick.put(parentNode, new ArrayList<>());
      if (!invalidNodesThisTick.containsKey(parentNode))
         invalidNodesThisTick.put(parentNode, new ArrayList<>());
      if (!nodesRejectedThisTick.containsKey(reason))
         nodesRejectedThisTick.put(reason, new ArrayList<>());

      validNodesThisTick.get(parentNode).remove(rejectedNode);
      invalidNodesThisTick.get(parentNode).add(rejectedNode);
      nodesRejectedThisTick.get(reason).add(rejectedNode);
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
      messager.submitMessage(FootstepPlannerMessagerAPI.ValidNodesThisTickTopic, validNodesThisTick);
      messager.submitMessage(FootstepPlannerMessagerAPI.InvalidNodesThisTickTopic, invalidNodesThisTick);
      messager.submitMessage(FootstepPlannerMessagerAPI.NodesRejectedThisTickTopic, nodesRejectedThisTick);

      validNodesThisTick.clear();
      invalidNodesThisTick.clear();
      nodesRejectedThisTick.clear();
   }
}

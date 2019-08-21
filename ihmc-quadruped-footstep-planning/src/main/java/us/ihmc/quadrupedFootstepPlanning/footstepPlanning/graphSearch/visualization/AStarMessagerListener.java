package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
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

   private final HashMap<FootstepNode, Pair<List<FootstepNode>, List<FootstepNode>>> nodesThisTick = new HashMap<>();
   private final HashMap<QuadrupedFootstepPlannerNodeRejectionReason, List<FootstepNode>> nodesRejectedThisTick = new HashMap<>();

   public AStarMessagerListener(Messager messager)
   {
      this.messager = messager;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (!nodesThisTick.containsKey(previousNode))
         nodesThisTick.put(previousNode, new ImmutablePair<>(new ArrayList<>(), new ArrayList<>()));

      nodesThisTick.get(previousNode).getLeft().add(node);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason reason)
   {
      if (!nodesThisTick.containsKey(parentNode))
      {
         nodesThisTick.put(parentNode, new ImmutablePair<>(new ArrayList<>(), new ArrayList<>()));
      }
      if (!nodesRejectedThisTick.containsKey(reason))
      {
         nodesRejectedThisTick.put(reason, new ArrayList<>());
      }

      nodesThisTick.get(parentNode).getLeft().remove(rejectedNode);
      nodesThisTick.get(parentNode).getRight().add(rejectedNode);
      nodesRejectedThisTick.get(reason).add(rejectedNode);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, QuadrupedFootstepPlannerNodeRejectionReason reason)
   {
      // TODO
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
      messager.submitMessage(FootstepPlannerMessagerAPI.NodesThisTickTopic, nodesThisTick);
      messager.submitMessage(FootstepPlannerMessagerAPI.NodesRejectedThisTickTopic, nodesRejectedThisTick);

      nodesThisTick.clear();
      nodesRejectedThisTick.clear();
   }
}

package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.visualization;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawStepPlannerListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PawStepAStarMessagerListener implements PawStepPlannerListener
{
   private final Messager messager;

   private final HashMap<PawNode, Pair<List<PawNode>, List<PawNode>>> nodesThisTick = new HashMap<>();
   private final HashMap<PawStepPlannerNodeRejectionReason, List<PawNode>> nodesRejectedThisTick = new HashMap<>();

   public PawStepAStarMessagerListener(Messager messager)
   {
      this.messager = messager;
   }

   @Override
   public void addNode(PawNode node, PawNode previousNode)
   {
      if (!nodesThisTick.containsKey(previousNode))
         nodesThisTick.put(previousNode, new ImmutablePair<>(new ArrayList<>(), new ArrayList<>()));

      nodesThisTick.get(previousNode).getLeft().add(node);
   }

   @Override
   public void rejectNode(PawNode rejectedNode, PawNode parentNode, PawStepPlannerNodeRejectionReason reason)
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
   public void rejectNode(PawNode rejectedNode, PawStepPlannerNodeRejectionReason reason)
   {
      // TODO
   }

   @Override
   public void plannerFinished(List<PawNode> plan)
   {

   }

   @Override
   public void reportLowestCostNodeList(List<PawNode> plan)
   {

   }

   @Override
   public void tickAndUpdate()
   {
      messager.submitMessage(PawStepPlannerMessagerAPI.NodesThisTickTopic, nodesThisTick);
      messager.submitMessage(PawStepPlannerMessagerAPI.NodesRejectedThisTickTopic, nodesRejectedThisTick);

      nodesThisTick.clear();
      nodesRejectedThisTick.clear();
   }
}

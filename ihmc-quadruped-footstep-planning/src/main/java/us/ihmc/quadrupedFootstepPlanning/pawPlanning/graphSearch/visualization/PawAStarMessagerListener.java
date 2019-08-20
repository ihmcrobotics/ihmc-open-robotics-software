package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.visualization;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.QuadrupedPawPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawPlannerListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PawAStarMessagerListener implements PawPlannerListener
{
   private final Messager messager;

   private final HashMap<PawNode, Pair<List<PawNode>, List<PawNode>>> nodesThisTick = new HashMap<>();
   private final HashMap<QuadrupedPawPlannerNodeRejectionReason, List<PawNode>> nodesRejectedThisTick = new HashMap<>();

   public PawAStarMessagerListener(Messager messager)
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
   public void rejectNode(PawNode rejectedNode, PawNode parentNode, QuadrupedPawPlannerNodeRejectionReason reason)
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
   public void rejectNode(PawNode rejectedNode, QuadrupedPawPlannerNodeRejectionReason reason)
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
      messager.submitMessage(PawPlannerMessagerAPI.NodesThisTickTopic, nodesThisTick);
      messager.submitMessage(PawPlannerMessagerAPI.NodesRejectedThisTickTopic, nodesRejectedThisTick);

      nodesThisTick.clear();
      nodesRejectedThisTick.clear();
   }
}

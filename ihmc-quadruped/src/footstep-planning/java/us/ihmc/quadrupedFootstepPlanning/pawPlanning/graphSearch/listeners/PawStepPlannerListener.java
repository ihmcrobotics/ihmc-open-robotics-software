package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners;


import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

import java.util.List;

public interface PawStepPlannerListener
{
   void addNode(PawNode node, PawNode previousNode);

   void rejectNode(PawNode rejectedNode, PawNode parentNode, PawStepPlannerNodeRejectionReason reason);

   void rejectNode(PawNode rejectedNode, PawStepPlannerNodeRejectionReason reason);

   void plannerFinished(List<PawNode> plan);

   void reportLowestCostNodeList(List<PawNode> plan);

   void tickAndUpdate();
}

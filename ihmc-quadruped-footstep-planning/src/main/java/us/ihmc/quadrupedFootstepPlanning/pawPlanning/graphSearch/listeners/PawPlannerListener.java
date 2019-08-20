package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners;


import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.QuadrupedPawPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

import java.util.List;

public interface PawPlannerListener
{
   void addNode(PawNode node, PawNode previousNode);

   void rejectNode(PawNode rejectedNode, PawNode parentNode, QuadrupedPawPlannerNodeRejectionReason reason);

   void rejectNode(PawNode rejectedNode, QuadrupedPawPlannerNodeRejectionReason reason);

   void plannerFinished(List<PawNode> plan);

   void reportLowestCostNodeList(List<PawNode> plan);

   void tickAndUpdate();
}

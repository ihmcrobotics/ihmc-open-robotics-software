package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class MessageBasedPlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;

   private final PlannerNodeDataList lowestNodeDataList = new PlannerNodeDataList();
   private final PlannerOccupancyMap occupancyMapSinceLastReport = new PlannerOccupancyMap();
   private final PlannerLatticeMap expandedNodesSinceLastReport = new PlannerLatticeMap();
   private final PlannerNodeDataList fullGraphSinceLastReport = new PlannerNodeDataList();

   private final long broadcastDt;
   private long lastBroadcastTime = -1;
   private final PlannerNodeDataList allNodes = new PlannerNodeDataList();


   public MessageBasedPlannerListener(FootstepNodeSnapperReadOnly snapper, long broadcastDt)
   {
      this.snapper = snapper;
      this.broadcastDt = broadcastDt;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      int previousNodeDataIndex;

      if (previousNode == null)
      {
         reset();
         previousNodeDataIndex = -1;
      }
      else
      {
         previousNodeDataIndex = allNodes.getNodeData().indexOf(previousNode.getLatticeNode());

         expandedNodesSinceLastReport.addLatticeNode(new LatticeNode(previousNode.getXIndex(), previousNode.getYIndex(), previousNode.getYawIndex()));
      }
      Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
      PlannerNodeData nodeData = allNodes.addNode(previousNodeDataIndex, allNodes.size(), node.getLatticeNode(), node.getRobotSide(), nodePose, null);

      fullGraphSinceLastReport.addNode(nodeData);
      occupancyMapSinceLastReport.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));
   }

   private void reset()
   {
      lowestNodeDataList.clear();
      allNodes.clear();
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
      lowestNodeDataList.clear();
      for (int i = 0; i < plan.size(); i++)
      {
         FootstepNode node = plan.get(i);
         Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
         lowestNodeDataList.addNode(i - 1, i, node.getLatticeNode(), node.getRobotSide(), nodePose, null);
      }
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      fullGraphSinceLastReport.getDataForNode(rejectedNode).setRejectionReason(reason);
   }

   @Override
   public void tickAndUpdate()
   {
      long currentTime = System.currentTimeMillis();

      if (lastBroadcastTime == -1)
         lastBroadcastTime = currentTime;

      if (currentTime - lastBroadcastTime > broadcastDt)
      {
         broadcastOccupancyMap(occupancyMapSinceLastReport);
         broadcastExpandedNodes(expandedNodesSinceLastReport);

         occupancyMapSinceLastReport.clear();
         expandedNodesSinceLastReport.clear();

         broadcastLowestCostNodeData(lowestNodeDataList);

         broadcastFullGraph(fullGraphSinceLastReport);
         fullGraphSinceLastReport.clear();

         lastBroadcastTime = currentTime;
      }
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      broadcastOccupancyMap(occupancyMapSinceLastReport);
   }

   abstract void broadcastOccupancyMap(PlannerOccupancyMap occupancyMap);

   abstract void broadcastExpandedNodes(PlannerLatticeMap latticeMap);

   abstract void broadcastLowestCostNodeData(PlannerNodeDataList nodeDataList);

   abstract void broadcastFullGraph(PlannerNodeDataList nodeDataList);
}

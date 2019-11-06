package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.HashMap;
import java.util.List;

public abstract class MessageBasedPlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapper snapper;

   private final PlannerNodeDataList lowestNodeDataList = new PlannerNodeDataList();
   private final PlannerOccupancyMap occupancyMapSinceLastReport = new PlannerOccupancyMap();
   private final PlannerLatticeMap expandedNodesSinceLastReport = new PlannerLatticeMap();
   private final PlannerNodeDataList fullGraphSinceLastReport = new PlannerNodeDataList();

   private final long broadcastDt;
   private long lastBroadcastTime = -1;
   private final HashMap<FootstepNode, PlannerNodeData> nodeDataMapSinceLastReport = new HashMap<>();

   private int totalNodeCount = 0;


   public MessageBasedPlannerListener(FootstepNodeSnapper snapper, long broadcastDt)
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
         previousNodeDataIndex = previousNode.getNodeIndex();
         expandedNodesSinceLastReport.addFootstepNode(previousNode);
      }
      RigidBodyTransform nodePose = snapper.snapFootstepNode(node).getOrComputeSnappedNodeTransform(node);

      node.setNodeIndex(totalNodeCount);
      PlannerNodeData nodeData = new PlannerNodeData(previousNodeDataIndex, node, nodePose, null);

      nodeDataMapSinceLastReport.put(node, nodeData);
      fullGraphSinceLastReport.addNode(nodeData);
      occupancyMapSinceLastReport.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));

      totalNodeCount++;
   }

   private void reset()
   {
      lowestNodeDataList.clear();
      nodeDataMapSinceLastReport.clear();
      totalNodeCount = 0;
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
      lowestNodeDataList.clear();
      for (int i = 0; i < plan.size(); i++)
      {
         FootstepNode node = plan.get(i);
         RigidBodyTransform nodePose = snapper.snapFootstepNode(node).getOrComputeSnappedNodeTransform(node);
         lowestNodeDataList.addNode(i - 1, i, node.getLatticeNode(), node.getRobotSide(), nodePose, null);
      }
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      nodeDataMapSinceLastReport.get(rejectedNode).setRejectionReason(reason);
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
         nodeDataMapSinceLastReport.clear();

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

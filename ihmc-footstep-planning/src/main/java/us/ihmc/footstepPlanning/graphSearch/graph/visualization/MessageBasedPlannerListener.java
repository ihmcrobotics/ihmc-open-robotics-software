package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class MessageBasedPlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapper snapper;

   private final PlannerNodeDataList lowestNodeDataList = new PlannerNodeDataList();
   private final PlannerOccupancyMap occupancyMapSinceLastReport = new PlannerOccupancyMap();

   private final long broadcastDt;
   private long lastBroadcastTime = -1;
   private final HashMap<FootstepNode, List<PlannerNodeData>> rejectedNodeData = new HashMap<>();

   private int totalNodeCount = 0;


   public MessageBasedPlannerListener(FootstepNodeSnapper snapper, long broadcastDt)
   {
      this.snapper = snapper;
      this.broadcastDt = broadcastDt;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
         reset();

      node.setNodeIndex(totalNodeCount);

      occupancyMapSinceLastReport.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));
      totalNodeCount++;
   }

   private void reset()
   {
      lowestNodeDataList.clear();
      rejectedNodeData.clear();

      totalNodeCount = 0;
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
      lowestNodeDataList.clear();
      for (int i = 0; i < plan.size(); i++)
      {
         FootstepNode node = plan.get(i);
         int parentNodeIndex = i > 0 ? plan.get(i - 1).getNodeIndex() : -1;
         RigidBodyTransform nodePose = snapper.snapFootstepNode(node).getOrComputeSnappedNodeTransform(node);
         lowestNodeDataList.addNode(parentNodeIndex, node, nodePose, null);
      }
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      RigidBodyTransform nodePose;
      if (reason != BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP && snapper != null)
      {
         nodePose = snapper.snapFootstepNode(rejectedNode).getOrComputeSnappedNodeTransform(rejectedNode);
      }
      else
      {
         nodePose = new RigidBodyTransform();
         nodePose.setTranslation(rejectedNode.getX(), rejectedNode.getY(), 0.0);
         nodePose.setRotationYaw(rejectedNode.getYaw());
      }
      PlannerNodeData nodeData = new PlannerNodeData(parentNode.getNodeIndex(), rejectedNode, nodePose, reason);

      if (rejectedNodeData.get(parentNode) == null)
         rejectedNodeData.put(parentNode, new ArrayList<>());
      rejectedNodeData.get(parentNode).add(nodeData);
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
         occupancyMapSinceLastReport.clear();

         broadcastLowestCostNodeData(lowestNodeDataList);

         lastBroadcastTime = currentTime;
      }
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      broadcastOccupancyMap(occupancyMapSinceLastReport);
   }

   @Override
   public HashMap<FootstepNode, List<PlannerNodeData>> getRejectedNodeData()
   {
      return rejectedNodeData;
   }

   abstract void broadcastOccupancyMap(PlannerOccupancyMap occupancyMap);

   abstract void broadcastLowestCostNodeData(PlannerNodeDataList nodeDataList);
}

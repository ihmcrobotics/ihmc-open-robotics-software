package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.idl.IDLSequence.Object;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public abstract class MessageBasedPlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final HashMap<FootstepNode, BipedalFootstepPlannerNodeRejectionReason> rejectionReasons = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> childMap = new HashMap<>();
   private final HashSet<PlannerCell> exploredCells = new HashSet<>();
   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();
   private final FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();

   private final long occupancyMapBroadcastDt;
   private long lastBroadcastTime = -1;

   public MessageBasedPlannerListener(FootstepNodeSnapperReadOnly snapper, long occupancyMapBroadcastDt)
   {
      this.snapper = snapper;
      this.occupancyMapBroadcastDt = occupancyMapBroadcastDt;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if(previousNode == null)
      {
         rejectionReasons.clear();
         childMap.clear();
         exploredCells.clear();
         lowestCostPlan.clear();
      }
      else
      {
         childMap.computeIfAbsent(previousNode, n -> new ArrayList<>()).add(node);
         exploredCells.add(new PlannerCell(node.getXIndex(), node.getYIndex()));
      }
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
      lowestCostPlan.clear();
      lowestCostPlan.addAll(plan);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      rejectionReasons.put(rejectedNode, reason);
   }

   @Override
   public void tickAndUpdate()
   {
      long currentTime = System.currentTimeMillis();

      if(lastBroadcastTime == -1)
         lastBroadcastTime = currentTime;

      if(currentTime - lastBroadcastTime > occupancyMapBroadcastDt)
      {
         packOccupancyMapMessage();
         broadcastOccupancyMap(occupancyMapMessage);

         if(packLowestCostPlanMessage())
            broadcastNodeData(nodeDataListMessage);

         lastBroadcastTime = currentTime;
      }
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      packOccupancyMapMessage();
      broadcastOccupancyMap(occupancyMapMessage);
   }

   abstract void broadcastOccupancyMap(FootstepPlannerOccupancyMapMessage occupancyMapMessage);

   abstract void broadcastNodeData(FootstepNodeDataListMessage nodeDataListMessage);

   private void packOccupancyMapMessage()
   {
      PlannerCell[] plannerCells = exploredCells.toArray(new PlannerCell[0]);
      Object<FootstepPlannerCellMessage> occupiedCells = occupancyMapMessage.getOccupiedCells();
      occupiedCells.clear();
      for (int i = 0; i < plannerCells.length; i++)
      {
         FootstepPlannerCellMessage plannerCell = occupiedCells.add();
         plannerCell.setXIndex(plannerCells[i].xIndex);
         plannerCell.setYIndex(plannerCells[i].yIndex);
      }
   }

   private boolean packLowestCostPlanMessage()
   {
      if(lowestCostPlan.isEmpty())
         return false;

      Object<FootstepNodeDataMessage> nodeDataList = nodeDataListMessage.getNodeData();
      nodeDataList.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         FootstepNodeDataMessage nodeDataMessage = nodeDataList.add();
         setNodeDataMessage(nodeDataMessage, node, -1);
      }

      nodeDataListMessage.setIsFootstepGraph(false);
      lowestCostPlan.clear();
      return true;
   }

   private void setNodeDataMessage(FootstepNodeDataMessage nodeDataMessage, FootstepNode node, int parentNodeIndex)
   {
      nodeDataMessage.setParentNodeId(parentNodeIndex);

      byte rejectionReason = rejectionReasons.containsKey(node) ? rejectionReasons.get(node).toByte() : (byte) 255;
      nodeDataMessage.setBipedalFootstepPlannerNodeRejectionReason(rejectionReason);

      nodeDataMessage.setRobotSide(node.getRobotSide().toByte());
      nodeDataMessage.setXIndex(node.getXIndex());
      nodeDataMessage.setYIndex(node.getYIndex());
      nodeDataMessage.setYawIndex(node.getYawIndex());

      FootstepNodeSnapData snapData = snapper.getSnapData(node);
      Point3D snapTranslationToSet = nodeDataMessage.getSnapTranslation();
      Quaternion snapRotationToSet = nodeDataMessage.getSnapRotation();
      snapData.getSnapTransform().get(snapRotationToSet, snapTranslationToSet);
   }

   private class PlannerCell
   {
      int xIndex;
      int yIndex;

      PlannerCell(int xIndex, int yIndex)
      {
         this.xIndex = xIndex;
         this.yIndex = yIndex;
      }

      @Override
      public int hashCode()
      {
         final int prime = 31;
         int result = 1;
         result = prime * result + xIndex;
         result = prime * result + yIndex;
         return result;
      }

      @Override
      public boolean equals(java.lang.Object other)
      {
         if(!(other instanceof PlannerCell))
            return false;

         PlannerCell otherCell = (PlannerCell) other;
         return otherCell.xIndex == xIndex && otherCell.yIndex == yIndex;
      }
   }
}

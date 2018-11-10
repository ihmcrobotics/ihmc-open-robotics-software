package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.idl.IDLSequence.Object;

import java.util.*;
import java.util.function.Supplier;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final HashMap<FootstepNode, BipedalFootstepPlannerNodeRejectionReason> rejectionReasons = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> childMap = new HashMap<>();
   private final HashSet<PlannerCell> exploredCells = new HashSet<>();
   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final ConcurrentList<FootstepPlannerCellMessage> occupiedCells = new ConcurrentList<>();
   private final ConcurrentList<FootstepNodeDataMessage> nodeData = new ConcurrentList<>();

   private final long occupancyMapBroadcastDt;
   private long lastBroadcastTime = -1;

   public StagePlannerListener(FootstepNodeSnapperReadOnly snapper, long occupancyMapBroadcastDt)
   {
      this.snapper = snapper;
      this.occupancyMapBroadcastDt = occupancyMapBroadcastDt;
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
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

      if (lastBroadcastTime == -1)
         lastBroadcastTime = currentTime;

      if (currentTime - lastBroadcastTime > occupancyMapBroadcastDt)
      {
         updateOccupiedCells();
         updateNodeData();
      }
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      updateOccupiedCells();
   }

   private void updateOccupiedCells()
   {
      occupiedCells.clear();
      PlannerCell[] plannerCells = exploredCells.toArray(new PlannerCell[0]);
      List<FootstepPlannerCellMessage> cellMessages = new ArrayList<>();
      for (int i = 0; i < plannerCells.length; i++)
      {
         FootstepPlannerCellMessage plannerCell = new FootstepPlannerCellMessage();
         plannerCell.setXIndex(plannerCells[i].xIndex);
         plannerCell.setYIndex(plannerCells[i].yIndex);
         cellMessages.add(plannerCell);
      }
      occupiedCells.addAll(cellMessages);
   }

   private void updateNodeData()
   {
      nodeData.clear();
      List<FootstepNodeDataMessage> messageList = new ArrayList<>();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         FootstepNodeDataMessage nodeDataMessage = new FootstepNodeDataMessage();
         setNodeDataMessage(nodeDataMessage, node, -1);
         messageList.add(nodeDataMessage);
      }
      nodeData.addAll(messageList);

      lowestCostPlan.clear();
   }

   FootstepPlannerOccupancyMapMessage packOccupancyMapMessage()
   {
      FootstepPlannerOccupancyMapMessage message = new FootstepPlannerOccupancyMapMessage();
      //      Object<FootstepPlannerCellMessage> occupiedCells = message.getOccupiedCells();
      //      for (int i = 0; i < this.occupiedCells.size(); i++)
      //         occupiedCells.add().set(this.occupiedCells.get(i));

      return message;
   }

   FootstepNodeDataListMessage packLowestCostPlanMessage()
   {
      FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();
      //      Object<FootstepNodeDataMessage> nodeDataList = nodeDataListMessage.getNodeData();
      //      for (int i = 0; i < nodeData.size(); i++)
      //         nodeDataList.add().set(nodeData.get(i));

      nodeDataListMessage.setIsFootstepGraph(false);

      return nodeDataListMessage;
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

   private class ConcurrentList<T> extends ConcurrentCopier<List<T>>
   {
      public ConcurrentList()
      {
         super(ArrayList::new);
      }

      public void clear()
      {
         getCopyForWriting().clear();
         commit();
      }

      public void add(T element)
      {
         List<T> currentSet = getCopyForReading();
         List<T> updatedSet = getCopyForWriting();
         updatedSet.clear();
         if (currentSet != null)
            updatedSet.addAll(currentSet);
         updatedSet.add(element);
         commit();
      }

      public void addAll(Collection<? extends T> collection)
      {
         List<T> currentSet = getCopyForReading();
         List<T> updatedSet = getCopyForWriting();
         updatedSet.clear();
         if (currentSet != null)
            updatedSet.addAll(currentSet);
         updatedSet.addAll(collection);
         commit();
      }

      public T[] toArray(T[] ts)
      {
         List<T> currentSet = getCopyForReading();
         return currentSet.toArray(ts);
      }

      public T get(int index)
      {
         return getCopyForReading().get(index);
      }

      public boolean isEmpty()
      {
         List<T> currentList = getCopyForReading();
         if (currentList == null)
            return true;

         return currentList.isEmpty();
      }

      public int size()
      {
         List<T> currentList = getCopyForReading();
         if (currentList == null)
            return 0;

         return currentList.size();
      }

   }
}

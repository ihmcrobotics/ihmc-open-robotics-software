package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.idl.IDLSequence.Object;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

import org.apache.commons.lang3.mutable.MutableInt;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final HashMap<FootstepNode, BipedalFootstepPlannerNodeRejectionReason> rejectionReasons = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> childMap = new HashMap<>();
   private final HashSet<PlannerCell> exploredCells = new HashSet<>();
   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();
   private final FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();

   private final RecyclingArrayList<FootstepNodeDataMessage> nodeDataMessageList = new RecyclingArrayList<>(FootstepNodeDataMessage::new);
   private final RecyclingArrayList<FootstepPlannerCellMessage> cellMessages = new RecyclingArrayList<>(FootstepPlannerCellMessage::new);

   private final ConcurrentList<FootstepPlannerCellMessage> occupiedCells = new ConcurrentList<>();
   private final ConcurrentList<FootstepNodeDataMessage> nodeData = new ConcurrentList<>();

   private final AtomicBoolean hasOccupiedCells = new AtomicBoolean(true);
   private final AtomicBoolean hasNodeData = new AtomicBoolean(true);

   private final long occupancyMapUpdateDt;
   private long lastUpdateTime = -1;
   private final EnumMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> rejectionCount = new EnumMap<>(BipedalFootstepPlannerNodeRejectionReason.class);
   private int totalNodeCount = 0;

   public StagePlannerListener(FootstepNodeSnapperReadOnly snapper, long occupancyMapUpdateDt)
   {
      this.snapper = snapper;
      this.occupancyMapUpdateDt = occupancyMapUpdateDt;

      for(BipedalFootstepPlannerNodeRejectionReason rejectionReason : BipedalFootstepPlannerNodeRejectionReason.values)
      {
         rejectionCount.put(rejectionReason, new MutableInt(0));
      }
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
      {
         reset();
      }
      else
      {
         childMap.computeIfAbsent(previousNode, n -> new ArrayList<>()).add(node);
         PlannerCell plannerCell = new PlannerCell(node.getXIndex(), node.getYIndex());

         exploredCells.add(plannerCell);
         totalNodeCount++;
      }
   }

   public void reset()
   {
      rejectionReasons.clear();
      childMap.clear();
      exploredCells.clear();
      lowestCostPlan.clear();
      totalNodeCount = 1;
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

      MutableInt rejectionCount = this.rejectionCount.get(reason);
      rejectionCount.setValue(rejectionCount.getValue() + 1);
   }

   @Override
   public void tickAndUpdate()
   {
      long currentTime = System.currentTimeMillis();

      if (lastUpdateTime == -1)
         lastUpdateTime = currentTime;

      boolean isTimeForUpdate = currentTime - lastUpdateTime > occupancyMapUpdateDt;
      if (!isTimeForUpdate)
         return;

      updateOccupiedCells();
      updateNodeData();

      lastUpdateTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      updateOccupiedCells();
   }

   private void updateOccupiedCells()
   {
      occupiedCells.clear();
      cellMessages.clear();
      PlannerCell[] plannerCells = exploredCells.toArray(new PlannerCell[0]);
      for (int i = 0; i < plannerCells.length; i++)
      {
         FootstepPlannerCellMessage plannerCell = cellMessages.add();
         plannerCell.setXIndex(plannerCells[i].xIndex);
         plannerCell.setYIndex(plannerCells[i].yIndex);
      }
      occupiedCells.addAll(cellMessages);

      hasOccupiedCells.set(true);
   }

   private void updateNodeData()
   {
      nodeData.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         FootstepNodeDataMessage nodeDataMessage = nodeDataMessageList.add();
         setNodeDataMessage(nodeDataMessage, node, -1);
      }
      nodeData.addAll(nodeDataMessageList);

      lowestCostPlan.clear();

      hasNodeData.set(true);
   }

   public boolean hasOccupiedCells()
   {
      return hasOccupiedCells.get();
   }

   public boolean hasNodeData()
   {
      return hasNodeData.get();
   }

   FootstepPlannerOccupancyMapMessage packOccupancyMapMessage()
   {
      if (occupiedCells.isEmpty())
         return null;

      Object<FootstepPlannerCellMessage> occupiedCellsForMessage = occupancyMapMessage.getOccupiedCells();
      occupiedCellsForMessage.clear();

      List<FootstepPlannerCellMessage> occupiedCells = this.occupiedCells.getCopyForReading();
      for (int i = 0; i < occupiedCells.size(); i++)
         occupiedCellsForMessage.add().set(occupiedCells.get(i));

      hasOccupiedCells.set(false);

      return occupancyMapMessage;
   }

   FootstepNodeDataListMessage packLowestCostPlanMessage()
   {
      if (nodeData.isEmpty())
         return null;

      Object<FootstepNodeDataMessage> nodeDataListForMessage = nodeDataListMessage.getNodeData();
      nodeDataListForMessage.clear();

      List<FootstepNodeDataMessage> nodeData = this.nodeData.getCopyForReading();
      for (int i = 0; i < nodeData.size(); i++)
         nodeDataListForMessage.add().set(nodeData.get(i));

      nodeDataListMessage.setIsFootstepGraph(false);

      hasNodeData.set(false);

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

      if(snapper != null)
      {
         FootstepNodeSnapData snapData = snapper.getSnapData(node);
         Point3D snapTranslationToSet = nodeDataMessage.getSnapTranslation();
         Quaternion snapRotationToSet = nodeDataMessage.getSnapRotation();
         snapData.getSnapTransform().get(snapRotationToSet, snapTranslationToSet);
      }
   }

   public int getTotalNodeCount()
   {
      return totalNodeCount;
   }

   public int getRejectionReasonCount(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return rejectionCount.get(rejectionReason).getValue();
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

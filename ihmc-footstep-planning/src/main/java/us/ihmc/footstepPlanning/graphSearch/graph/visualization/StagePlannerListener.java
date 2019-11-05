package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
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
   private final PlannerOccupancyMap occupancyMapSinceLastReport = new PlannerOccupancyMap();
   private final PlannerLatticeMap latticeMapSinceLastReport = new PlannerLatticeMap();
   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();

   private final RecyclingArrayList<FootstepNodeDataMessage> nodeDataMessageList = new RecyclingArrayList<>(FootstepNodeDataMessage::new);

   private final ConcurrentList<FootstepNodeDataMessage> nodeData = new ConcurrentList<>();
   private final ConcurrentCopier<PlannerOccupancyMap> concurrentOccupancyMap = new ConcurrentCopier<>(PlannerOccupancyMap::new);
   private final ConcurrentCopier<PlannerLatticeMap> concurrentLatticeMap = new ConcurrentCopier<>(PlannerLatticeMap::new);

   private final AtomicBoolean hasOccupiedCells = new AtomicBoolean(true);
   private final AtomicBoolean hasLatticeMap = new AtomicBoolean(true);
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
         occupancyMapSinceLastReport.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));
         latticeMapSinceLastReport.addLatticeNode(new LatticeNode(node.getXIndex(), node.getYIndex(), node.getYawIndex()));
         totalNodeCount++;
      }
   }

   public void reset()
   {
      rejectionReasons.clear();
      childMap.clear();
      occupancyMapSinceLastReport.clear();
      latticeMapSinceLastReport.clear();
      lowestCostPlan.clear();
      totalNodeCount = 1;
      rejectionCount.values().forEach(count -> count.setValue(0));
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
      rejectionCount.get(reason).increment();
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
      updateLatticeMap();
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
      PlannerOccupancyMap concurrentOccupancyMap = this.concurrentOccupancyMap.getCopyForReading();
      concurrentOccupancyMap.set(occupancyMapSinceLastReport);
      this.concurrentOccupancyMap.commit();
      occupancyMapSinceLastReport.clear();

      hasOccupiedCells.set(!concurrentOccupancyMap.getOccupiedCells().isEmpty());
   }

   private void updateLatticeMap()
   {
      PlannerLatticeMap concurrentLatticeMap = this.concurrentLatticeMap.getCopyForReading();
      concurrentLatticeMap.set(latticeMapSinceLastReport);
      this.concurrentLatticeMap.commit();
      latticeMapSinceLastReport.clear();

      hasLatticeMap.set(!concurrentLatticeMap.getLatticeNodes().isEmpty());
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

   public boolean hasLatticeMap()
   {
      return hasLatticeMap.get();
   }

   public boolean hasNodeData()
   {
      return hasNodeData.get();
   }

   PlannerOccupancyMap getOccupancyMap()
   {
      hasOccupiedCells.set(false);

      return concurrentOccupancyMap.getCopyForReading();
   }

   PlannerLatticeMap getLatticeMap()
   {
      hasLatticeMap.set(false);

      return concurrentLatticeMap.getCopyForReading();
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
      byte rejectionReason = rejectionReasons.containsKey(node) ? rejectionReasons.get(node).toByte() : (byte) 255;

      nodeDataMessage.setParentNodeId(parentNodeIndex);
      nodeDataMessage.setBipedalFootstepPlannerNodeRejectionReason(rejectionReason);
      nodeDataMessage.setRobotSide(node.getRobotSide().toByte());

      if (snapper != null)
      {
         Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
         nodeDataMessage.getPosition().set(nodePose.getPosition());
         nodeDataMessage.getOrientation().set(nodePose.getOrientation());
      }
      else
      {
         nodeDataMessage.getPosition().set(node.getX(), node.getY(), 0.0);
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

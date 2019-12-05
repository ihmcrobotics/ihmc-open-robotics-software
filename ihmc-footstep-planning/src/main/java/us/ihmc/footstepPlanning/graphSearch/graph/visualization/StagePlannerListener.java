package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.log.LogTools;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapper snapper;

   private final HashSet<PlannerCell> occupancyMapCellsThisTick = new HashSet<>();
   private final ConcurrentSet<PlannerCell> occupancyMapCellsSinceLastReportReference = new ConcurrentSet<>();

   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final HashMap<FootstepNode, List<PlannerNodeData>> rejectedNodeData = new HashMap<>();

   private final ConcurrentCopier<PlannerNodeDataList> concurrentLowestCostNodeDataList = new ConcurrentCopier<>(PlannerNodeDataList::new);

   private final AtomicBoolean hasOccupiedCells = new AtomicBoolean(false);
   private final AtomicBoolean hasLowestCostPlan = new AtomicBoolean(false);

   private final long occupancyMapUpdateDt;
   private long lastUpdateTime = -1;
   private final EnumMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> rejectionCount = new EnumMap<>(BipedalFootstepPlannerNodeRejectionReason.class);
   private int totalNodeCount = 0;

   public StagePlannerListener(FootstepNodeSnapper snapper, long occupancyMapUpdateDt)
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
         reset();

      node.setNodeIndex(totalNodeCount);

      occupancyMapCellsThisTick.add(new PlannerCell(node.getXIndex(), node.getYIndex()));
      totalNodeCount++;
   }

   public void reset()
   {
      rejectedNodeData.clear();

      occupancyMapCellsSinceLastReportReference.clear();

      lowestCostPlan.clear();
      totalNodeCount = 0;
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
      if (parentNode == null)
      {
         LogTools.warn("Rejecting the initial node, because the parent is null.");
         return;
      }

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
      updateLowestCostPlan();

      lastUpdateTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      updateOccupiedCells();
   }

   private void updateOccupiedCells()
   {
      occupancyMapCellsSinceLastReportReference.addAll(occupancyMapCellsThisTick);
      hasOccupiedCells.set(hasOccupiedCells.get() || !occupancyMapCellsThisTick.isEmpty());
      occupancyMapCellsThisTick.clear();
   }

   private void updateLowestCostPlan()
   {
      PlannerNodeDataList concurrentNodeDataList = this.concurrentLowestCostNodeDataList.getCopyForWriting();
      concurrentNodeDataList.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         RigidBodyTransform nodePose;
         if (snapper == null)
            nodePose = new RigidBodyTransform(new Quaternion(node.getYaw(), 0.0, 0.0), new Vector3D(node.getX(), node.getY(), 0.0));
         else
            nodePose = snapper.snapFootstepNode(node).getOrComputeSnappedNodeTransform(node);
         int parentNodeIndex = i > 0 ? lowestCostPlan.get(i - 1).getNodeIndex() : -1;
         concurrentNodeDataList.addNode(parentNodeIndex, node, nodePose, null);
      }
      this.concurrentLowestCostNodeDataList.commit();
      lowestCostPlan.clear();

      hasLowestCostPlan.set(!concurrentNodeDataList.getNodeData().isEmpty());
   }

   public boolean hasOccupiedCells()
   {
      return hasOccupiedCells.get();
   }

   public boolean hasLowestCostPlan()
   {
      return hasLowestCostPlan.get();
   }

   PlannerOccupancyMap getOccupancyMap()
   {
      PlannerOccupancyMap occupancyMap = new PlannerOccupancyMap();
      for (PlannerCell plannerCell : occupancyMapCellsSinceLastReportReference.getCopyForReading())
         occupancyMap.addOccupiedCell(plannerCell);

      hasOccupiedCells.set(false);
      occupancyMapCellsSinceLastReportReference.clear();

      return occupancyMap;
   }

   PlannerNodeDataList getLowestCostPlan()
   {
      hasLowestCostPlan.set(false);

      return concurrentLowestCostNodeDataList.getCopyForReading();
   }

   public int getTotalNodeCount()
   {
      return totalNodeCount;
   }

   public int getRejectionReasonCount(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return rejectionCount.get(rejectionReason).getValue();
   }

   @Override
   public HashMap<FootstepNode, List<PlannerNodeData>> getRejectedNodeData()
   {
      return rejectedNodeData;
   }

   private class ConcurrentSet<T> extends ConcurrentCopier<Set<T>>
   {
      public ConcurrentSet()
      {
         super(HashSet::new);
      }

      public void clear()
      {
         getCopyForWriting().clear();
         commit();
      }

      public void addAll(Collection<? extends T> collection)
      {
         Set<T> currentSet = getCopyForReading();
         Set<T> updatedSet = getCopyForWriting();
         updatedSet.clear();
         if (currentSet != null)
            updatedSet.addAll(currentSet);
         updatedSet.addAll(collection);
         commit();
      }

      public T[] toArray(T[] ts)
      {
         Set<T> currentSet = getCopyForReading();
         return currentSet.toArray(ts);
      }

      public boolean isEmpty()
      {
         Set<T> currentList = getCopyForReading();
         if (currentList == null)
            return true;

         return currentList.isEmpty();
      }

      public int size()
      {
         Set<T> currentList = getCopyForReading();
         if (currentList == null)
            return 0;

         return currentList.size();
      }
   }
}

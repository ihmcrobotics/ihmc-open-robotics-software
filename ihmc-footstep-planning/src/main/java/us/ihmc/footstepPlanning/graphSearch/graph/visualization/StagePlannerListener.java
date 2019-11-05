package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;

   private final ConcurrentList<PlannerCell> occupancyMapCellsSinceLastReport = new ConcurrentList<>();
   private final ConcurrentList<LatticeNode> expandedNodesSinceLastReport = new ConcurrentList<>();
   private final ConcurrentList<PlannerNodeData> fullGraphSinceLastReport = new ConcurrentList<>();

   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();


   private final List<PlannerNodeData> incomingNodeDataThisTick = new ArrayList<>();
   private final List<PlannerCell> incomingOccupiedCellsThisTick = new ArrayList<>();
   private final List<LatticeNode> incomingExpandedNodes = new ArrayList<>();

   private final PlannerNodeDataList allNodes = new PlannerNodeDataList();

   private final ConcurrentCopier<PlannerNodeDataList> concurrentLowestCostNodeDataList = new ConcurrentCopier<>(PlannerNodeDataList::new);

   private final AtomicBoolean hasOccupiedCells = new AtomicBoolean(true);
   private final AtomicBoolean hasExpandedNodes = new AtomicBoolean(true);
   private final AtomicBoolean hasFullGraph = new AtomicBoolean(true);
   private final AtomicBoolean hasLowestCostPlan = new AtomicBoolean(true);

   private final long occupancyMapUpdateDt;
   private long lastUpdateTime = -1;
   private final EnumMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> rejectionCount = new EnumMap<>(BipedalFootstepPlannerNodeRejectionReason.class);
   private int totalNodeCount = 0;

   public StagePlannerListener(FootstepNodeSnapperReadOnly snapper, long occupancyMapUpdateDt)
   {
      this.snapper = snapper;
      this.occupancyMapUpdateDt = occupancyMapUpdateDt;
      allNodes.setIsFootstepGraph(true);

      for(BipedalFootstepPlannerNodeRejectionReason rejectionReason : BipedalFootstepPlannerNodeRejectionReason.values)
      {
         rejectionCount.put(rejectionReason, new MutableInt(0));
      }
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

         incomingExpandedNodes.add(new LatticeNode(previousNode.getXIndex(), previousNode.getYIndex(), previousNode.getYawIndex()));
      }

      Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
      PlannerNodeData nodeData = allNodes.addNode(previousNodeDataIndex, allNodes.size(), node.getLatticeNode(), node.getRobotSide(), nodePose, null);

      incomingNodeDataThisTick.add(nodeData);
      incomingOccupiedCellsThisTick.add(new PlannerCell(node.getXIndex(), node.getYIndex()));

      totalNodeCount++;
   }

   public void reset()
   {
      allNodes.clear();
      incomingNodeDataThisTick.clear();
      incomingOccupiedCellsThisTick.clear();
      incomingExpandedNodes.clear();

      occupancyMapCellsSinceLastReport.clear();
      expandedNodesSinceLastReport.clear();

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
      PlannerNodeData nodeData = incomingNodeDataThisTick.get(incomingNodeDataThisTick.indexOf(rejectedNode.getLatticeNode()));
      nodeData.setRejectionReason(reason);

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
      updateExpandedNodes();
      updateLowestCostPlan();
      updateFullGraph();

      lastUpdateTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      updateOccupiedCells();
   }

   private void updateOccupiedCells()
   {
      occupancyMapCellsSinceLastReport.addAll(incomingOccupiedCellsThisTick);
      hasOccupiedCells.set(hasOccupiedCells.get() || !incomingOccupiedCellsThisTick.isEmpty());

      incomingOccupiedCellsThisTick.clear();
   }

   private void updateExpandedNodes()
   {
      expandedNodesSinceLastReport.addAll(incomingExpandedNodes);
      hasExpandedNodes.set(hasExpandedNodes.get() || !incomingExpandedNodes.isEmpty());

      incomingExpandedNodes.clear();
   }

   private void updateLowestCostPlan()
   {
      PlannerNodeDataList concurrentNodeDataList = this.concurrentLowestCostNodeDataList.getCopyForWriting();
      concurrentNodeDataList.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
         concurrentNodeDataList.addNode(-1, i, node.getLatticeNode(), node.getRobotSide(), nodePose, null);
      }
      this.concurrentLowestCostNodeDataList.commit();
      lowestCostPlan.clear();

      hasLowestCostPlan.set(!concurrentNodeDataList.getNodeData().isEmpty());
   }

   private void updateFullGraph()
   {
      fullGraphSinceLastReport.addAll(incomingNodeDataThisTick);
      hasFullGraph.set(hasFullGraph.get() || !incomingNodeDataThisTick.isEmpty());

      incomingNodeDataThisTick.clear();
   }

   public boolean hasOccupiedCells()
   {
      return hasOccupiedCells.get();
   }

   public boolean hasExpandedNodes()
   {
      return hasExpandedNodes.get();
   }

   public boolean hasLowestCostPlan()
   {
      return hasLowestCostPlan.get();
   }

   public boolean hasFullGraph()
   {
      return hasFullGraph.get();
   }

   PlannerOccupancyMap getOccupancyMap()
   {
      PlannerOccupancyMap occupancyMap = new PlannerOccupancyMap();
      for (PlannerCell plannerCell : occupancyMapCellsSinceLastReport.getCopyForReading())
         occupancyMap.addOccupiedCell(plannerCell);

      hasOccupiedCells.set(false);
      occupancyMapCellsSinceLastReport.clear();

      return occupancyMap;
   }

   PlannerLatticeMap getExpandedNodes()
   {
      PlannerLatticeMap latticeMap = new PlannerLatticeMap();
      for (LatticeNode latticeNode : expandedNodesSinceLastReport.getCopyForReading())
         latticeMap.addLatticeNode(latticeNode);

      hasExpandedNodes.set(false);
      expandedNodesSinceLastReport.clear();

      return latticeMap;
   }

   PlannerNodeDataList getLowestCostPlan()
   {
      hasLowestCostPlan.set(false);

      return concurrentLowestCostNodeDataList.getCopyForReading();
   }

   PlannerNodeDataList getFullGraph()
   {
      PlannerNodeDataList plannerNodeDataList = new PlannerNodeDataList();
      plannerNodeDataList.setIsFootstepGraph(true);

      for (PlannerNodeData nodeData : fullGraphSinceLastReport.getCopyForReading())
         plannerNodeDataList.addNode(nodeData);

      hasFullGraph.set(false);
      fullGraphSinceLastReport.clear();

      return plannerNodeDataList;
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

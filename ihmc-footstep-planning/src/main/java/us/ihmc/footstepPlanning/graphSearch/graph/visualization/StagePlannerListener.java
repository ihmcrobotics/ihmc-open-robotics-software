package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepEdge;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapper snapper;

   private final AtomicReference<Collection<PlannerCell>> occupancyMapCellsSinceLastReportReference = new AtomicReference<>();
   private final AtomicReference<Collection<FootstepNode>> expandedNodesSinceLastReportReference = new AtomicReference<>();
   private final AtomicReference<List<PlannerNodeData>> fullGraphToReportReference = new AtomicReference<>();

   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final List<PlannerNodeData> rejectedNodeData = new ArrayList<>();

   private final ConcurrentCopier<PlannerNodeDataList> concurrentLowestCostNodeDataList = new ConcurrentCopier<>(PlannerNodeDataList::new);

   private final AtomicBoolean hasOccupiedCells = new AtomicBoolean(false);
   private final AtomicBoolean hasExpandedNodes = new AtomicBoolean(false);
   private final AtomicBoolean hasFullGraph = new AtomicBoolean(false);
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

      occupancyMapCellsSinceLastReportReference.get().add(new PlannerCell(node.getXIndex(), node.getYIndex()));
      totalNodeCount++;
   }

   public void reset()
   {
      rejectedNodeData.clear();

      occupancyMapCellsSinceLastReportReference.set(new HashSet<>());
      expandedNodesSinceLastReportReference.set(new HashSet<>());
      fullGraphToReportReference.set(new ArrayList<>());

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
      RigidBodyTransform nodePose = snapper.snapFootstepNode(rejectedNode).getOrComputeSnappedNodeTransform(rejectedNode);
      PlannerNodeData nodeData = new PlannerNodeData(parentNode.getNodeIndex(), rejectedNode, nodePose, reason);

      rejectedNodeData.add(nodeData);
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
//      updateLowestCostPlan();

      lastUpdateTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan, Collection<FootstepNode> expandedNodes, FootstepGraph footstepGraph)
   {
      updateOccupiedCells();

      Collection<FootstepNode> expandedNodesToSet = expandedNodesSinceLastReportReference.get();
      expandedNodesToSet.clear();
      expandedNodesToSet.addAll(expandedNodes);

      computeFullGraphData(footstepGraph);
   }

   private void updateOccupiedCells()
   {
      hasOccupiedCells.set(!occupancyMapCellsSinceLastReportReference.get().isEmpty());
   }

   private void updateLowestCostPlan()
   {
      PlannerNodeDataList concurrentNodeDataList = this.concurrentLowestCostNodeDataList.getCopyForWriting();
      concurrentNodeDataList.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         RigidBodyTransform nodePose = snapper.snapFootstepNode(node).getOrComputeSnappedNodeTransform(node);
         concurrentNodeDataList.addNode(-1, i, node.getLatticeNode(), node.getRobotSide(), nodePose, null);
      }
      this.concurrentLowestCostNodeDataList.commit();
      lowestCostPlan.clear();

      hasLowestCostPlan.set(!concurrentNodeDataList.getNodeData().isEmpty());
   }

   private void computeFullGraphData(FootstepGraph footstepGraph)
   {
      List<PlannerNodeData> fullGraphToReport = this.fullGraphToReportReference.get();
      HashMap<FootstepNode, HashSet<FootstepEdge>> outgoingEdges = footstepGraph.getOutgoingEdges();
      for (FootstepNode footstepNode : outgoingEdges.keySet())
      {
         for (FootstepEdge outgoingEdge : outgoingEdges.get(footstepNode))
         {
            FootstepNode childNode = outgoingEdge.getEndNode();
            RigidBodyTransform nodePose = snapper.snapFootstepNode(childNode).getOrComputeSnappedNodeTransform(childNode);
            PlannerNodeData nodeData = new PlannerNodeData(footstepNode.getNodeIndex(), childNode, nodePose, null);
            fullGraphToReport.add(nodeData);
         }
      }
      fullGraphToReport.addAll(rejectedNodeData);

      hasFullGraph.set(true);
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
      for (PlannerCell plannerCell : occupancyMapCellsSinceLastReportReference.getAndSet(new HashSet<>()))
         occupancyMap.addOccupiedCell(plannerCell);

      hasOccupiedCells.set(false);

      return occupancyMap;
   }

   PlannerLatticeMap getExpandedNodes()
   {
      PlannerLatticeMap latticeMap = new PlannerLatticeMap();
      for (FootstepNode latticeNode : expandedNodesSinceLastReportReference.getAndSet(new HashSet<>()))
         latticeMap.addFootstepNode(latticeNode);

      hasExpandedNodes.set(false);

      return latticeMap;
   }

   PlannerNodeDataList getLowestCostPlan()
   {
      hasLowestCostPlan.set(false);

      return concurrentLowestCostNodeDataList.getCopyForReading();
   }

   PlannerNodeDataList getFullGraph()
   {
      if (!hasFullGraph.get())
         return null;

      PlannerNodeDataList plannerNodeDataList = new PlannerNodeDataList();
      plannerNodeDataList.setIsFootstepGraph(true);

      for (PlannerNodeData nodeData : fullGraphToReportReference.getAndSet(new ArrayList<>()))
         plannerNodeDataList.addNode(nodeData);

      hasFullGraph.set(false);

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

}

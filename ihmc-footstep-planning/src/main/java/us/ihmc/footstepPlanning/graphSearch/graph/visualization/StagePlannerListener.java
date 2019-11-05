package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final HashMap<FootstepNode, BipedalFootstepPlannerNodeRejectionReason> rejectionReasons = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> childMap = new HashMap<>();
   private final PlannerOccupancyMap occupancyMapSinceLastReport = new PlannerOccupancyMap();
   private final PlannerLatticeMap latticeMapSinceLastReport = new PlannerLatticeMap();
   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final ConcurrentCopier<PlannerNodeDataList> concurrentLowestCostNodeDataList = new ConcurrentCopier<>(PlannerNodeDataList::new);
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
      PlannerOccupancyMap concurrentOccupancyMap = this.concurrentOccupancyMap.getCopyForWriting();
      concurrentOccupancyMap.set(occupancyMapSinceLastReport);
      this.concurrentOccupancyMap.commit();
      occupancyMapSinceLastReport.clear();

      hasOccupiedCells.set(!concurrentOccupancyMap.getOccupiedCells().isEmpty());
   }

   private void updateLatticeMap()
   {
      PlannerLatticeMap concurrentLatticeMap = this.concurrentLatticeMap.getCopyForWriting();
      concurrentLatticeMap.set(latticeMapSinceLastReport);
      this.concurrentLatticeMap.commit();
      latticeMapSinceLastReport.clear();

      hasLatticeMap.set(!concurrentLatticeMap.getLatticeNodes().isEmpty());
   }

   private void updateNodeData()
   {
      PlannerNodeDataList concurrentNodeDataList = this.concurrentLowestCostNodeDataList.getCopyForWriting();
      concurrentNodeDataList.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
         concurrentNodeDataList.addNode(-1, node.getRobotSide(), nodePose, null);
      }
      this.concurrentLowestCostNodeDataList.commit();
      lowestCostPlan.clear();

      hasNodeData.set(!concurrentNodeDataList.getNodeData().isEmpty());
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

   PlannerNodeDataList getLowestCostPlan()
   {
      hasNodeData.set(false);

      return concurrentLowestCostNodeDataList.getCopyForReading();
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
}

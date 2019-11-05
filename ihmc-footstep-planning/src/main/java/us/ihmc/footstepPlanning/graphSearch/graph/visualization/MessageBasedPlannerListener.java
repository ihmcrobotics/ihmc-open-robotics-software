package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class MessageBasedPlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final HashMap<FootstepNode, BipedalFootstepPlannerNodeRejectionReason> rejectionReasons = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> childMap = new HashMap<>();
   private final PlannerNodeDataList lowestNodeDataList = new PlannerNodeDataList();
   private final PlannerOccupancyMap occupancyMapSinceLastReport = new PlannerOccupancyMap();
   private final PlannerLatticeMap expandedNodesSinceLastReport = new PlannerLatticeMap();

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
      if (previousNode == null)
      {
         rejectionReasons.clear();
         childMap.clear();
         lowestNodeDataList.clear();
      }
      else
      {
         childMap.computeIfAbsent(previousNode, n -> new ArrayList<>()).add(node);
         occupancyMapSinceLastReport.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));
         expandedNodesSinceLastReport.addLatticeNode(new LatticeNode(previousNode.getXIndex(), previousNode.getYIndex(), previousNode.getYawIndex()));
      }
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
      lowestNodeDataList.clear();
      for (int i = 0; i < plan.size(); i++)
      {
         FootstepNode node = plan.get(i);
         Pose3DReadOnly nodePose = FootstepNodeTools.getNodePoseInWorld(node, snapper.getSnapData(node).getSnapTransform());
         lowestNodeDataList.addNode(i - 1, node.getRobotSide(), nodePose, null);
      }
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
         broadcastOccupancyMap(occupancyMapSinceLastReport);
         broadcastExpandedNodes(expandedNodesSinceLastReport);

         occupancyMapSinceLastReport.clear();
         expandedNodesSinceLastReport.clear();

         broadcastNodeData(lowestNodeDataList);

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

   abstract void broadcastNodeData(PlannerNodeDataList nodeDataList);
}

package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
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

public class MultiStagePlannerListener implements BipedalFootstepPlannerListener
{
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();
   private final FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();

   private final long occupancyMapBroadcastDt;
   private long lastBroadcastTime = -1;

   private final List<StagePlannerListener> listeners = new ArrayList<>();

   public MultiStagePlannerListener(StatusMessageOutputManager statusMessageOutputManager, long occupancyMapBroadcastDt)
   {
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.occupancyMapBroadcastDt = occupancyMapBroadcastDt;
   }

   public long getBroadcastDt()
   {
      return occupancyMapBroadcastDt;
   }

   public void addStagePlannerListener(StagePlannerListener listener)
   {
      listeners.add(listener);
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
   }

   @Override
   public void tickAndUpdate()
   {
      long currentTime = System.currentTimeMillis();

      if (lastBroadcastTime == -1)
         lastBroadcastTime = currentTime;

      boolean isTimeForBroadcast = currentTime - lastBroadcastTime > occupancyMapBroadcastDt;
      if (!isTimeForBroadcast)
         return;

      boolean areListenersUpdated = true;
      for (StagePlannerListener listener : listeners)
         areListenersUpdated &= listener.hasNodeData() && listener.hasOccupiedCells();

      if (!areListenersUpdated)
         return;

      Object<FootstepPlannerCellMessage> occupiedCells = occupancyMapMessage.getOccupiedCells();
      Object<FootstepNodeDataMessage> nodeData = nodeDataListMessage.getNodeData();
      occupiedCells.clear();
      nodeData.clear();

      for (StagePlannerListener listener : listeners)
      {
         FootstepPlannerOccupancyMapMessage stageOccupancyMap = listener.packOccupancyMapMessage();
         if (stageOccupancyMap != null)
         {
            Object<FootstepPlannerCellMessage> stageOccupiedCells = stageOccupancyMap.getOccupiedCells();
            for (int i = 0; i < stageOccupiedCells.size(); i++)
               occupiedCells.add().set(stageOccupiedCells.get(i));
         }

         FootstepNodeDataListMessage stageNodeList = listener.packLowestCostPlanMessage();
         if (stageNodeList != null)
         {
            Object<FootstepNodeDataMessage> stageNodeData = stageNodeList.getNodeData();
            for (int i = 0; i < stageNodeData.size(); i++)
               nodeData.add().set(stageNodeData.get(i));
         }
      }

      broadcastOccupancyMap(occupancyMapMessage);

      if (!nodeData.isEmpty())
         broadcastNodeData(nodeDataListMessage);

      lastBroadcastTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();
      for (StagePlannerListener listener : listeners)
      {
         FootstepPlannerOccupancyMapMessage stageOccupancyMap = listener.packOccupancyMapMessage();
         if (stageOccupancyMap != null)
         {
            for (int i = 0; i < stageOccupancyMap.getOccupiedCells().size(); i++)
               occupancyMapMessage.getOccupiedCells().add().set(stageOccupancyMap.getOccupiedCells().get(i));
         }
      }
      broadcastOccupancyMap(occupancyMapMessage);
   }

   private void broadcastNodeData(FootstepNodeDataListMessage message)
   {
//      statusMessageOutputManager.reportStatusMessage(message);
   }

   private void broadcastOccupancyMap(FootstepPlannerOccupancyMapMessage message)
   {
//      statusMessageOutputManager.reportStatusMessage(message);
   }

}

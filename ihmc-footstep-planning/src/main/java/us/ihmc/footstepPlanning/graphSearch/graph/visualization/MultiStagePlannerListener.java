package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.*;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.idl.IDLSequence.Object;

import java.util.ArrayList;
import java.util.List;

public class MultiStagePlannerListener
{
   private final FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();
   private final FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();

   private final long occupancyMapBroadcastDt;
   private long lastBroadcastTime = -1;

   private final List<StagePlannerListener> listeners = new ArrayList<>();

   public MultiStagePlannerListener(long occupancyMapBroadcastDt)
   {
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

   public void packPlannerStatistics(FootstepPlanningStatistics planningStatistics)
   {
      int numberOfStepsConsidered = 0;
      for (int i = 0; i < listeners.size(); i++)
      {
         numberOfStepsConsidered += listeners.get(i).getTotalNodeCount();
      }
      planningStatistics.setNumberOfStepsConsidered(numberOfStepsConsidered);

      int totalRejectionCount = 0;
      int[] rejectionCountArray = new int[BipedalFootstepPlannerNodeRejectionReason.values.length];
      for (BipedalFootstepPlannerNodeRejectionReason rejectionReason : BipedalFootstepPlannerNodeRejectionReason.values)
      {
         for (int i = 0; i < listeners.size(); i++)
         {
            int rejectionCount = listeners.get(i).getRejectionReasonCount(rejectionReason);
            rejectionCountArray[rejectionReason.ordinal()] += rejectionCount;
            totalRejectionCount += rejectionCount;
         }
      }

      planningStatistics.setPercentageOfRejectedSteps(((double) totalRejectionCount) / numberOfStepsConsidered);
      planningStatistics.getRejectionPercentages().fill(0, BipedalFootstepPlannerNodeRejectionReason.values.length, 0.0);

      if(totalRejectionCount > 0)
      {
         for (int i = 0; i < BipedalFootstepPlannerNodeRejectionReason.values.length; i++)
         {
            double rejectionPercentage = ((double) rejectionCountArray[i]) / totalRejectionCount;
            planningStatistics.getRejectionPercentages().set(i, rejectionPercentage);
         }
      }
   }

   public void reset()
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         listeners.get(i).reset();
      }
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

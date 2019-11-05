package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class MultiStagePlannerListener
{
   private final FootstepNodeDataListMessage nodeDataListMessage = new FootstepNodeDataListMessage();

   private final long occupancyMapBroadcastDt;
   private long lastBroadcastTime = -1;

   private final List<StagePlannerListener> listeners = new ArrayList<>();

   private final StatusMessageOutputManager statusOutputManager;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean broadcastOccupancyMap = new YoBoolean("broadcastOccupancyMap", registry);

   public MultiStagePlannerListener(StatusMessageOutputManager statusOutputManager, long occupancyMapBroadcastDt, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;
      this.occupancyMapBroadcastDt = occupancyMapBroadcastDt;

      parentRegistry.addChild(registry);
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

      boolean occupiedCellsUpToDate = true;
      boolean nodeDataIsUpToDate = true;
      for (StagePlannerListener listener : listeners)
      {
         nodeDataIsUpToDate &= listener.hasNodeData();
         occupiedCellsUpToDate &= listener.hasOccupiedCells();
      }

      if (occupiedCellsUpToDate && nodeDataIsUpToDate)
         return;

      if (broadcastOccupancyMap.getBooleanValue())
      {
         broadcastOccupancyMap(getConcatenatedOccupancyMap());
      }


         Object<FootstepNodeDataMessage> nodeData = nodeDataListMessage.getNodeData();
         nodeData.clear();

         for (StagePlannerListener listener : listeners)
         {
            FootstepNodeDataListMessage stageNodeList = listener.packLowestCostPlanMessage();
            if (stageNodeList != null)
            {
               Object<FootstepNodeDataMessage> stageNodeData = stageNodeList.getNodeData();
               for (int i = 0; i < stageNodeData.size(); i++)
                  nodeData.add().set(stageNodeData.get(i));
            }
         }

         if (!nodeData.isEmpty())
            broadcastNodeData(nodeDataListMessage);

      lastBroadcastTime = currentTime;
   }

   public void plannerFinished(List<FootstepNode> plan)
   {
      broadcastOccupancyMap(getConcatenatedOccupancyMap());
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

      planningStatistics.setFractionOfRejectedSteps(((double) totalRejectionCount) / numberOfStepsConsidered);
      planningStatistics.getRejectionFractions().fill(0, BipedalFootstepPlannerNodeRejectionReason.values.length, 0.0);

      if(totalRejectionCount > 0)
      {
         for (int i = 0; i < BipedalFootstepPlannerNodeRejectionReason.values.length; i++)
         {
            double rejectionPercentage = ((double) rejectionCountArray[i]) / totalRejectionCount;
            planningStatistics.getRejectionFractions().set(i, rejectionPercentage);
         }
      }
   }

   private FootstepPlannerOccupancyMapMessage getConcatenatedOccupancyMap()
   {
      FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();
      for (StagePlannerListener listener : listeners)
      {
         for (PlannerCell stageCell : listener.getOccupancyMap().getOccupiedCells())
            stageCell.getAsMessage(occupancyMapMessage.getOccupiedCells().add());
      }
      return occupancyMapMessage;
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
//      statusOutputManager.reportStatusMessage(message);
   }

   private void broadcastOccupancyMap(FootstepPlannerOccupancyMapMessage message)
   {
      statusOutputManager.reportStatusMessage(message);
   }

}

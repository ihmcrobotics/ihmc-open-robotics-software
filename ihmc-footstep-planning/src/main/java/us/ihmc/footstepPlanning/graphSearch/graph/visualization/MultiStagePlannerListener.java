package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class MultiStagePlannerListener
{
   private final long occupancyMapBroadcastDt;
   private long lastBroadcastTime = -1;

   private final List<StagePlannerListener> listeners = new ArrayList<>();

   private final StatusMessageOutputManager statusOutputManager;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean broadcastOccupancyMap = new YoBoolean("broadcastOccupancyMap", registry);
   private final YoBoolean broadcastLowestCostPlan = new YoBoolean("broadcastLowestCostPlan", registry);

   public MultiStagePlannerListener(StatusMessageOutputManager statusOutputManager, long occupancyMapBroadcastDt, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;
      this.occupancyMapBroadcastDt = occupancyMapBroadcastDt;

      broadcastOccupancyMap.set(true);

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

      boolean stageHasMapUpdate = false;
      boolean stageHasLowestCostPlan = false;

      for (StagePlannerListener listener : listeners)
      {
         stageHasMapUpdate |= listener.hasOccupiedCells();
         stageHasLowestCostPlan |= listener.hasLowestCostPlan();
      }

      if (stageHasMapUpdate)
      {
         if (broadcastOccupancyMap.getBooleanValue())
         {
            FootstepPlannerOccupancyMapMessage message = getConcatenatedOccupancyMap();
            statusOutputManager.reportStatusMessage(message);
         }

         lastBroadcastTime = currentTime;
      }

      if (stageHasLowestCostPlan && broadcastLowestCostPlan.getBooleanValue())
      {
//         FootstepNodeDataListMessage message = getConcatenatedLowestCostNodeData();
//         if (!message.getNodeData().isEmpty())
//            statusOutputManager.reportStatusMessage(message);
      }
   }

   public void plannerFinished(List<FootstepNode> plan)
   {
      if (broadcastOccupancyMap.getBooleanValue())
      {
         FootstepPlannerOccupancyMapMessage message = getConcatenatedOccupancyMap();
         statusOutputManager.reportStatusMessage(message);
      }
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
         //         if (!listener.hasOccupiedCells())
         //            continue;

         PlannerOccupancyMap occupancyMap = listener.getOccupancyMap();
         if (occupancyMap == null)
            continue;

         for (PlannerCell stageCell : occupancyMap.getOccupiedCells())
         {
            if (occupancyMap.getOccupiedCells().size() == 10000)
               break;
            stageCell.getAsMessage(occupancyMapMessage.getOccupiedCells().add());
         }
      }

      return occupancyMapMessage;
   }

   private FootstepNodeDataListMessage getConcatenatedLowestCostNodeData()
   {
      FootstepNodeDataListMessage message = new FootstepNodeDataListMessage();
      message.setIsFootstepGraph(false);
      for (StagePlannerListener listener : listeners)
      {
         if (!listener.hasLowestCostPlan())
            continue;

         PlannerNodeDataList lowestCostPlan = listener.getLowestCostPlan();
         if (lowestCostPlan == null)
            continue;

         for (PlannerNodeData nodeData : lowestCostPlan.getNodeData())
            nodeData.getAsMessage(message.getNodeData().add());
      }
      return message;
   }

   public void reset()
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         listeners.get(i).reset();
      }
   }
}

package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
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
   private final YoBoolean broadcastLatticeMap = new YoBoolean("broadcastLatticeMap", registry);

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

      boolean isUpToDate = true;
      for (StagePlannerListener listener : listeners)
         isUpToDate &= listener.hasNodeData() && listener.hasOccupiedCells() && listener.hasLatticeMap();

      if (isUpToDate)
         return;

      if (broadcastOccupancyMap.getBooleanValue())
         statusOutputManager.reportStatusMessage(getConcatenatedOccupancyMap());

      if (broadcastLatticeMap.getBooleanValue())
         statusOutputManager.reportStatusMessage(getConcatenatedLatticeMap());


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
         statusOutputManager.reportStatusMessage(nodeDataListMessage);

      lastBroadcastTime = currentTime;
   }

   public void plannerFinished(List<FootstepNode> plan)
   {
      if (broadcastOccupancyMap.getBooleanValue())
         statusOutputManager.reportStatusMessage(getConcatenatedOccupancyMap());
      if (broadcastLatticeMap.getBooleanValue())
         statusOutputManager.reportStatusMessage(getConcatenatedLatticeMap());
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

   private FootstepPlannerLatticeMapMessage getConcatenatedLatticeMap()
   {
      FootstepPlannerLatticeMapMessage latticeMapMessage = new FootstepPlannerLatticeMapMessage();
      for (StagePlannerListener listener : listeners)
      {
         for (LatticeNode stageCell : listener.getLatticeMap().getLatticeNodes())
         {
            FootstepPlannerLatticeNodeMessage nodeMessage = latticeMapMessage.getLatticeNodes().add();
            nodeMessage.setXIndex(stageCell.getXIndex());
            nodeMessage.setYIndex(stageCell.getYIndex());
            nodeMessage.setYawIndex(stageCell.getYawIndex());
         }
      }
      return latticeMapMessage;
   }

   public void reset()
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         listeners.get(i).reset();
      }
   }
}

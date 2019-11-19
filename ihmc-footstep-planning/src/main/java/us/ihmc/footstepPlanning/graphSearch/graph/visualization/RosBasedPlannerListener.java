package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;

public class RosBasedPlannerListener extends MessageBasedPlannerListener
{
   private final StatusMessageOutputManager statusMessageOutputManager;

   public RosBasedPlannerListener(StatusMessageOutputManager statusMessageOutputManager, FootstepNodeSnapper snapper, long broadcastDtMillis)
   {
      super(snapper, broadcastDtMillis);
      this.statusMessageOutputManager = statusMessageOutputManager;
   }

   @Override
   void broadcastLowestCostNodeData(PlannerNodeDataList message)
   {
      statusMessageOutputManager.reportStatusMessage(message.getAsMessage());
   }

   @Override
   void broadcastOccupancyMap(PlannerOccupancyMap message)
   {
      statusMessageOutputManager.reportStatusMessage(message.getAsMessage());
   }
}

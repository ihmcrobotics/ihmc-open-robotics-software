package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.messager.Messager;

public class MessagerBasedPlannerListener extends MessageBasedPlannerListener
{
   private final Messager messager;

   public MessagerBasedPlannerListener(Messager messager, FootstepNodeSnapper snapper, long broadcastDtMillis)
   {
      super(snapper, broadcastDtMillis);
      this.messager = messager;
   }

   @Override
   void broadcastLowestCostNodeData(PlannerNodeDataList message)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.NodeData, message);
   }

   @Override
   void broadcastOccupancyMap(PlannerOccupancyMap message)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMap, message);
   }
}

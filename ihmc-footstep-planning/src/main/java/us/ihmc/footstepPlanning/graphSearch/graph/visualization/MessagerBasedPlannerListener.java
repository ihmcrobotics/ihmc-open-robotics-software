package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.messager.Messager;

public class MessagerBasedPlannerListener extends MessageBasedPlannerListener
{
   private final Messager messager;

   public MessagerBasedPlannerListener(Messager messager, FootstepNodeSnapperReadOnly snapper, long broadcastDtMillis)
   {
      super(snapper, broadcastDtMillis);
      this.messager = messager;
   }

   @Override
   void broadcastNodeData(FootstepNodeDataListMessage message)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.NodeData, message);
   }

   @Override
   void broadcastOccupancyMap(FootstepPlannerOccupancyMapMessage message)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMap, message);
   }
}

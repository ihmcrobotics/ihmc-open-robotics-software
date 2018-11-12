package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.ros2.RealtimeRos2Node;

public class RosBasedPlannerListener extends MessageBasedPlannerListener
{
   private final StatusMessageOutputManager statusMessageOutputManager;

   public RosBasedPlannerListener(StatusMessageOutputManager statusMessageOutputManager, FootstepNodeSnapperReadOnly snapper, long broadcastDtMillis)
   {
      super(snapper, broadcastDtMillis);
      this.statusMessageOutputManager = statusMessageOutputManager;
   }

   @Override
   void broadcastNodeData(FootstepNodeDataListMessage message)
   {
//      statusMessageOutputManager.reportStatusMessage(message);
   }

   @Override
   void broadcastOccupancyMap(FootstepPlannerOccupancyMapMessage message)
   {
//      statusMessageOutputManager.reportStatusMessage(message);
   }
}

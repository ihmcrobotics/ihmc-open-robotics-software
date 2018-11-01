package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.ros2.RealtimeRos2Node;

public class RosBasedPlannerListener extends MessageBasedPlannerListener
{
   private final IHMCRealtimeROS2Publisher<FootstepNodeDataListMessage> nodeDataListPublisher;
   private final IHMCRealtimeROS2Publisher<FootstepPlannerOccupancyMapMessage> occupancyMapPublisher;

   public RosBasedPlannerListener(RealtimeRos2Node node, FootstepNodeSnapperReadOnly snapper, long broadcastDtMillis)
   {
      super(snapper, broadcastDtMillis);
      this.nodeDataListPublisher = ROS2Tools.createPublisher(node, FootstepNodeDataListMessage.class, ROS2Tools::generateDefaultTopicName);
      this.occupancyMapPublisher = ROS2Tools.createPublisher(node, FootstepPlannerOccupancyMapMessage.class, ROS2Tools::generateDefaultTopicName);
   }

   @Override
   void broadcastNodeData(FootstepNodeDataListMessage message)
   {
      nodeDataListPublisher.publish(message);
   }

   @Override
   void broadcastOccupancyMap(FootstepPlannerOccupancyMapMessage message)
   {
      occupancyMapPublisher.publish(message);
   }
}

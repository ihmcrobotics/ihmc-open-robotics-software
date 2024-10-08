package us.ihmc.perception.streaming;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

import java.util.HashSet;
import java.util.Set;

public class ROS2SRTVideoStreamImageMessageRelay
{
   private final ROS2Node loopbackNode = ROS2Tools.createLoopbackROS2Node(PubSubImplementation.FAST_RTPS, "srt_stream_image_message_republisher");

   private final Set<ROS2SRTVideoStreamImageMessageRelayWorker> workers = new HashSet<>();

   public ROS2SRTVideoStreamImageMessageRelay(Set<ROS2SRTStreamTopicPair> topicsToRelay, ROS2Node ros2Node)
   {
      for (ROS2SRTStreamTopicPair topicPair : topicsToRelay)
         workers.add(new ROS2SRTVideoStreamImageMessageRelayWorker(loopbackNode, ros2Node, topicPair));
   }

   public void destroy()
   {
      for (ROS2SRTVideoStreamImageMessageRelayWorker worker : workers)
         worker.destroy();

      loopbackNode.destroy();
   }
}

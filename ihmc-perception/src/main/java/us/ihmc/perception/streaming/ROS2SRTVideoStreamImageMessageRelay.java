package us.ihmc.perception.streaming;

import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.jros2.ROS2Node;

import java.util.HashSet;
import java.util.Set;

public class ROS2SRTVideoStreamImageMessageRelay
{
   private final Set<ROS2SRTVideoStreamImageMessageRelayWorker> workers = new HashSet<>();
   private final ROS2Node relayNode = new ROS2Node("srt_relay_node");

   public ROS2SRTVideoStreamImageMessageRelay(Set<ROS2SRTStreamTopicPair> topicsToRelay, ROS2Node ros2Node, CompressionType compressionType)
   {
      for (ROS2SRTStreamTopicPair topicPair : topicsToRelay)
         workers.add(new ROS2SRTVideoStreamImageMessageRelayWorker(relayNode, ros2Node, topicPair, compressionType));
   }

   public void destroy()
   {
      for (ROS2SRTVideoStreamImageMessageRelayWorker worker : workers)
         worker.destroy();

      relayNode.close();
   }
}

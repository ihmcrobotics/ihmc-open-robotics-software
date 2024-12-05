package us.ihmc.perception.streaming;

import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.ros2.ROS2Node;

import java.util.HashSet;
import java.util.Set;

public class ROS2SRTVideoStreamImageMessageRelay
{
   private final Set<ROS2SRTVideoStreamImageMessageRelayWorker> workers = new HashSet<>();

   public ROS2SRTVideoStreamImageMessageRelay(Set<ROS2SRTStreamTopicPair> topicsToRelay, ROS2Node visualizerNode, ROS2Node otherNode, CompressionType compressionType)
   {
      for (ROS2SRTStreamTopicPair topicPair : topicsToRelay)
         workers.add(new ROS2SRTVideoStreamImageMessageRelayWorker(visualizerNode, otherNode, topicPair, compressionType));
   }

   public void destroy()
   {
      for (ROS2SRTVideoStreamImageMessageRelayWorker worker : workers)
         worker.destroy();
   }
}

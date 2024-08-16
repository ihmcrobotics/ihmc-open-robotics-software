package us.ihmc.communication;

import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

public class ROS2TopicHz
{
   public static void main(String[] args)
   {
      ROS2Topic<BigVideoPacket> topic = PerceptionAPI.BIG_VIDEO_TEST;
      LogTools.info("Subscribing to {}", topic.toString());
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter();
      ROS2Node node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "hz");
      new ROS2Callback<>(node, topic, message -> hz.ping());

      ThreadTools.sleepForever();
   }
}

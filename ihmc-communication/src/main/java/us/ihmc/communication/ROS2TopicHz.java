package us.ihmc.communication;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

public class ROS2TopicHz
{
   public static void main(String[] args)
   {
      ROS2Topic<?> topic = ROS2Tools.D435_POINT_CLOUD;
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter();
      ROS2Node node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "hz");
      new IHMCROS2Callback<>(node, topic, ROS2QosProfile.DEFAULT(), message -> hz.ping());

      ThreadTools.sleepForever();
   }
}

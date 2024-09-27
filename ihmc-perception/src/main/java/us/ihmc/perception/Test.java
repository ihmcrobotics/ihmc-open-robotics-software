package us.ihmc.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

public class Test
{
   public static void main(String[] args)
   {
      ROS2Node node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "asdfasdf");
   }
}

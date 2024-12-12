package us.ihmc.communication.ros2.sync;

import org.junit.jupiter.api.Test;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

public class ROS2DistributedClockTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.INTRAPROCESS, "test");

      new ROS2DistributedClock(ros2Node);
   }
}

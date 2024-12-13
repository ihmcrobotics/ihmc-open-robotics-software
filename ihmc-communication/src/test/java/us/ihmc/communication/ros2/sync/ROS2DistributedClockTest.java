package us.ihmc.communication.ros2.sync;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2NodeBuilder.SpecialTransportMode;

public class ROS2DistributedClockTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY).build("test");

      new ROS2DistributedClock(ros2Node);
      new ROS2DistributedClock(ros2Node);

      ros2Node = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY).build("test2");

      new ROS2DistributedClock(ros2Node);
      new ROS2DistributedClock(ros2Node);

      ThreadTools.park(1.0);
   }
}

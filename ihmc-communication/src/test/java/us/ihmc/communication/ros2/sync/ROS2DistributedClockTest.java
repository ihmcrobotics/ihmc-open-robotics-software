package us.ihmc.communication.ros2.sync;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2NodeBuilder.SpecialTransportMode;

import java.time.Duration;

public class ROS2DistributedClockTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node1 = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY).build("test");

      ROS2DistributedClock clock1 = new ROS2DistributedClock(ros2Node1);

      ROS2Node ros2Node2 = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY).build("test2");

      ROS2DistributedClock clock2 = new ROS2DistributedClock(ros2Node2);

      ThreadTools.park(1.0);

      Assertions.assertEquals(1, clock1.getPeerList().size());
      Assertions.assertEquals(1, clock2.getPeerList().size());

      Duration offset0 = clock1.getPeerList().get(0).getPeerClockOffset();
      LogTools.info("Clock 1 offset: {}", offset0);
      Duration offset1 = clock2.getPeerList().get(0).getPeerClockOffset();
      LogTools.info("Clock 2 offset: {}", offset1);

      Assertions.assertTrue(TimeTools.toDoubleSeconds(offset0) < 0.1);
      Assertions.assertTrue(TimeTools.toDoubleSeconds(offset1) < 0.1);

      clock1.destroy();
      clock2.destroy();

      ros2Node1.destroy();
      ros2Node2.destroy();
   }
}

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
import java.time.Instant;

public class ROS2DistributedClockTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node0 = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY).build("distributed_clock_test0");
      ROS2Node ros2Node1 = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY).build("distributed_clock_test1");

      ROS2DistributedClock[] clocks = new ROS2DistributedClock[3];
      clocks[0] = new ROS2DistributedClock(ros2Node0);
      clocks[1] = new ROS2DistributedClock(ros2Node0);
      clocks[2] = new ROS2DistributedClock(ros2Node1);

      ThreadTools.park(0.5);

      for (int i = 0; i < 10; i++)
      {
         for (ROS2DistributedClock clock : clocks)
            Assertions.assertEquals(clocks.length - 1, clock.getPeerList().size());

         for (ROS2DistributedClock clock : clocks)
         {
            Duration offset = clock.getPeerList().get(0).getPeerClockOffset();
            LogTools.info("Clock offset: {}", offset);
            Assertions.assertTrue(TimeTools.toDoubleSeconds(offset) < 0.1);
         }

         for (ROS2DistributedClock clock : clocks)
         {
            LogTools.info("Converted peer time: {}", clock.getPeerList().get(0).convertPeerTimeToOurTime(Instant.now()));
         }

         ThreadTools.park(0.3);
      }

      for (ROS2DistributedClock clock : clocks)
         clock.destroy();

      ros2Node0.destroy();
      ros2Node1.destroy();
   }
}

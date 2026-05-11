package us.ihmc.communication.ros2.sync;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;

@Disabled // TODO: jros2 migration - ROS2NodeBuilder not ported yet
public class ROS2PeerClockOffsetEstimatorTest
{
   @Test
   @Timeout(30)
   public void test()
   {
      ROS2Node ros2Node0 = new ROS2Node("peer_clock_test0");
      ROS2Node ros2Node1 = new ROS2Node("peer_clock_test1");

      ROS2PeerClockOffsetEstimator[] clocks = new ROS2PeerClockOffsetEstimator[3];
      clocks[0] = new ROS2PeerClockOffsetEstimator(ros2Node0);
      clocks[1] = new ROS2PeerClockOffsetEstimator(ros2Node0);
      clocks[2] = new ROS2PeerClockOffsetEstimator(ros2Node1);

      boolean peersPopulated = false;
      while (!peersPopulated)
      {
         peersPopulated = true;
         for (ROS2PeerClockOffsetEstimator clock : clocks)
            peersPopulated = clock.getPeerList().size() == clocks.length - 1;

         ThreadTools.park(0.1);
      }

      for (ROS2PeerClockOffsetEstimator clock : clocks)
         clock.getPeerList().get(0).getUpdatedNotification().blockingPoll();

      for (int i = 0; i < 10; i++)
      {
         for (ROS2PeerClockOffsetEstimator clock : clocks)
            Assertions.assertEquals(clocks.length - 1, clock.getPeerList().size());

         for (ROS2PeerClockOffsetEstimator clock : clocks)
         {
            Assertions.assertTrue(clock.getPeerList().get(0).isAlive());
            Duration offset = clock.getPeerList().get(0).getPeerClockOffset();
            LogTools.info("Clock offset: {}", offset);
            Assertions.assertTrue(TimeTools.toDoubleSeconds(offset) < 0.1);
         }

         for (ROS2PeerClockOffsetEstimator clock : clocks)
         {
            LogTools.info("Converted peer time: {}", clock.getPeerList().get(0).getPeerTimeInLocalFrame(Instant.now()));
         }

         ThreadTools.park(0.3);
      }

      for (ROS2PeerClockOffsetEstimator clock : clocks)
         clock.destroy();

      ros2Node0.close();
      ros2Node1.close();
   }
}

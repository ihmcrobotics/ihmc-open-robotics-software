package us.ihmc.perception.streaming;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;

import java.net.InetSocketAddress;

import static java.lang.Thread.interrupted;
import static org.junit.jupiter.api.Assertions.*;

public class ROS2StreamStatusMonitorTest
{
   private static final ROS2Node ROS2_NODE = new ROS2NodeBuilder().build("stream_status_monitor_test_node");
   private static final ROS2Helper ROS2_HELPER = new ROS2Helper(ROS2_NODE);
   private static final ROS2Topic<SRTStreamStatus> TEST_TOPIC = new ROS2Topic<SRTStreamStatus>().withSuffix("srt_status_test").withType(SRTStreamStatus.class);

   @AfterAll
   public static void destroyNode()
   {
      ROS2_NODE.destroy();
   }

   @AfterEach
   public void waitABit()
   {
      // Messages being sent over ROS2 from previous test may mess up the next test,
      // so we wait a bit after running each test
      ThreadTools.park(1.0);
   }

   @Test
   public void testSendAndReceive() throws InterruptedException
   {
      float messagePublishFrequency = 30.0f;
      float messagePublishPeriod = (float) Conversions.hertzToSeconds(messagePublishFrequency);

      InetSocketAddress streamerAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60001);
      int imageWidth = 1280;
      int imageHeight = 720;

      SRTStreamStatus streamStatusMessage = new SRTStreamStatus();
      streamStatusMessage.setStreamerAddress(streamerAddress.getHostString());
      streamStatusMessage.setStreamerPort(streamerAddress.getPort());
      streamStatusMessage.setImageWidth(imageWidth);
      streamStatusMessage.setImageHeight(imageHeight);
      streamStatusMessage.setIsStreaming(true);
      streamStatusMessage.setExpectedPublishFrequency(messagePublishFrequency);

      ROS2StreamStatusMonitor streamStatusMonitor = new ROS2StreamStatusMonitor(ROS2_HELPER, TEST_TOPIC);

      Throttler messagePublishThrottler = new Throttler();
      messagePublishThrottler.setFrequency(messagePublishFrequency);

      assertFalse(streamStatusMonitor.isStreaming());

      Thread messagePublishThread = ThreadTools.startAsDaemon(() ->
      {
         // Publish messages for 1 second
         for (float i = 0.0f; i < messagePublishFrequency; i++)
         {
            messagePublishThrottler.waitAndRun();
            ROS2_HELPER.publish(TEST_TOPIC, streamStatusMessage);
         }
      }, getClass().getSimpleName() + "Thread");

      // Wait for messages to arrive
      streamStatusMonitor.waitForStream(1.0);

      assertTrue(streamStatusMonitor.isStreaming());
      assertEquals(streamerAddress, streamStatusMonitor.getStreamerAddress());
      assertEquals(imageWidth, streamStatusMonitor.getCameraIntrinsics().getWidth());
      assertEquals(imageHeight, streamStatusMonitor.getCameraIntrinsics().getHeight());

      messagePublishThread.join();

      assertTrue(streamStatusMonitor.isStreaming());

      ThreadTools.park(ROS2StreamStatusMonitor.MESSAGE_EXPIRATION_MULTIPLIER * messagePublishPeriod);
      ThreadTools.park(0.05); // Sleep a little extra

      assertFalse(streamStatusMonitor.isStreaming());
   }

   @Test
   public void testStreamingStatus() throws InterruptedException
   {
      float messagePublishFrequency = 30.0f;
      Throttler messagePublishThrottler = new Throttler();
      messagePublishThrottler.setFrequency(messagePublishFrequency);

      SRTStreamStatus statusMessage = new SRTStreamStatus();
      statusMessage.setIsStreaming(true);

      ROS2StreamStatusMonitor streamStatusMonitor = new ROS2StreamStatusMonitor(ROS2_HELPER, TEST_TOPIC);
      assertFalse(streamStatusMonitor.isStreaming());

      Thread messagePublishThread = ThreadTools.startAsDaemon(() ->
      {
         // Publish messages for 3 second
         while (!interrupted())
         {
            ROS2_HELPER.publish(TEST_TOPIC, statusMessage);
         }
         // Publish end stream message
         statusMessage.setIsStreaming(false);
         ROS2_HELPER.publish(TEST_TOPIC, statusMessage);
      }, getClass().getSimpleName() + "Thread");

      streamStatusMonitor.waitForStream(1.5);
      assertTrue(streamStatusMonitor.isStreaming());

      messagePublishThread.interrupt();
      messagePublishThread.join();
      ThreadTools.park(0.5);
      assertFalse(streamStatusMonitor.isStreaming());
   }
}

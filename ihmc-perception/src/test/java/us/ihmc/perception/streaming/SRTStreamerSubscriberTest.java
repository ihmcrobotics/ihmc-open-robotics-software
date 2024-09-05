package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.SRTStreamRequest;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImageTest;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.Throttler;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;
import static org.junit.jupiter.api.Assertions.*;

public class SRTStreamerSubscriberTest
{
   private static final ROS2Node ROS2_NODE = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_streaming_test");
   private static final ROS2Helper ROS2_HELPER = new ROS2Helper(ROS2_NODE);
   private static final double FPS = 30.0;

   private static Mat sampleImage;

   @BeforeAll
   public static void readSampleImage() throws URISyntaxException, IOException
   {
      byte[] imageBytes = Files.readAllBytes(Path.of(RawImageTest.class.getResource("zedColorBGR.raw").toURI()));
      sampleImage = new Mat(720, 1280, opencv_core.CV_8UC3, new BytePointer(imageBytes));
   }

   @AfterAll
   public static void closeSampleImage()
   {
      sampleImage.close();
   }

   @Test
   public void testSRTStreamerAndSubscriber() throws InterruptedException
   {
      InetSocketAddress localAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60001);
      Throttler throttler = new Throttler();
      throttler.setFrequency(FPS);

      // Initialize the streamer
      SRTVideoStreamer streamer = new SRTVideoStreamer();
      streamer.initialize(sampleImage.cols(), sampleImage.rows(), FPS, AV_PIX_FMT_BGR24);
      assertEquals(0, streamer.connectedCallerCount());

      // In a separate thread, wait for the subscriber to connect
      Thread streamerConnectThread = ThreadTools.startAThread(() ->
      {
         // Wait for caller to connect
         streamer.connectToCaller(localAddress);

         // Send 2 seconds of video
         for (int i = 0; i < 2 * FPS; ++i)
         {
            throttler.waitAndRun();
            streamer.sendFrame(sampleImage);
         }
      }, "SRTStreamerTestConnection");

      // Initialize a subscriber and connect to streamer
      SRTVideoSubscriber subscriber = new SRTVideoSubscriber(localAddress, AV_PIX_FMT_BGR24);
      assertFalse(subscriber.isConnected());
      subscriber.connect();

      // Should be communicating now
      assertEquals(1, streamer.connectedCallerCount());
      assertTrue(subscriber.isConnected());

      // Ensure we can receive an image
      Mat receivedImage = subscriber.getNextImage(0.5);
      assertNotNull(receivedImage);
      receivedImage.close();

      streamerConnectThread.join();

      streamer.destroy();
      subscriber.destroy();

      // No communication
      assertEquals(0, streamer.connectedCallerCount());
      assertFalse(subscriber.isConnected());
   }

   @Test
   public void testROS2SRTStreamerAndSubscriber() throws InterruptedException
   {
      InetSocketAddress localAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60002);
      Throttler throttler = new Throttler();
      throttler.setFrequency(FPS);

      // ROS2 topic for the streamer and subscriber
      ROS2Topic<SRTStreamRequest> requestTopic = PerceptionAPI.STREAMING_MODULE.withSuffix("ros2_srt_test").withType(SRTStreamRequest.class);

      // Create and initialize the streamer
      ROS2SRTVideoStreamer streamer = new ROS2SRTVideoStreamer(requestTopic);
      streamer.initialize(sampleImage.cols(), sampleImage.rows(), FPS, AV_PIX_FMT_BGR24);

      // Create the subscriber
      ROS2SRTVideoSubscriber subscriber = new ROS2SRTVideoSubscriber(ROS2_HELPER, requestTopic, localAddress, AV_PIX_FMT_BGR24);

      // No communication at this time
      assertEquals(0, streamer.connectedCallerCount());
      assertFalse(subscriber.isConnected());

      Thread sendThread = ThreadTools.startAThread(() ->
      {
         // Send 2 seconds of video
         for (int i = 0; i < 2 * FPS; ++i)
         {
            throttler.waitAndRun();
            streamer.sendFrame(sampleImage);
         }
      }, "ROS2SRTStreamerSend");

      // Try subscribing
      subscriber.subscribe();

      // Should be communicating now
      assertTrue(subscriber.isConnected());
      assertEquals(1, streamer.connectedCallerCount());

      // Ensure we can receive an image
      Mat receivedImage = subscriber.getNextImage(0.5);
      assertNotNull(receivedImage);
      receivedImage.close();

      sendThread.join();

      // Destroy everything
      streamer.destroy();
      subscriber.destroy();

      // No communication
      assertEquals(0, streamer.connectedCallerCount());
      assertFalse(subscriber.isConnected());
   }
}

package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.SRTStreamRequest;
import us.ihmc.commons.Conversions;
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
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;
import static org.junit.jupiter.api.Assertions.*;

public class SRTStreamerSubscriberTest
{
   private static final ROS2Node ROS2_NODE = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_streaming_test");
   private static final ROS2Helper ROS2_HELPER = new ROS2Helper(ROS2_NODE);
   private static final double FPS = 30.0;
   private static final double TEST_TIMEOUT = 5.0;
   private static final double CALL_TIMEOUT = 0.5 * TEST_TIMEOUT;
   private static final double EXTRA_LONG_TIMEOUT = 2.0 * TEST_TIMEOUT;

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
   public void testSRTStreamerTimeouts()
   {
      InetSocketAddress fakeAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60001);

      SRTVideoStreamer streamer = new SRTVideoStreamer();
      streamer.initialize(sampleImage.cols(), sampleImage.rows(), FPS, AV_PIX_FMT_BGR24);

      testTimeout(() -> streamer.connectToCaller(fakeAddress, CALL_TIMEOUT), EXTRA_LONG_TIMEOUT);  // Try connecting to nowhere
      testTimeout(() -> streamer.sendFrame(sampleImage), TEST_TIMEOUT);                            // Try sending data to nowhere
      testTimeout(() -> streamer.removeCaller(fakeAddress), TEST_TIMEOUT);                         // Try removing nothing
      testTimeout(streamer::destroy, TEST_TIMEOUT);                                                // Try destroying

      // If it got here, the test passed!
   }

   @Test
   public void testSRTSubscriberTimeouts()
   {
      double TIMEOUT = 5.0;
      InetSocketAddress subscriberAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60001);

      SRTVideoSubscriber subscriber = new SRTVideoSubscriber(subscriberAddress, AV_PIX_FMT_BGR24);
      testTimeout(() -> subscriber.connect(CALL_TIMEOUT), TIMEOUT);
      testTimeout(() -> subscriber.waitForConnection(CALL_TIMEOUT), TIMEOUT); // Try waiting for a connection to nowhere
      testTimeout(() -> subscriber.getNextImage(CALL_TIMEOUT), TIMEOUT);      // Try to get an image from nowhere
      testTimeout(subscriber::destroy, TIMEOUT);                              // Try to destroy
   }

   private static final ExecutorService TIMEOUT_TEST_EXECUTOR = Executors.newSingleThreadExecutor();

   @AfterAll
   public static void shutdownExecutor()
   {
      TIMEOUT_TEST_EXECUTOR.shutdownNow();
   }

   private void testTimeout(Runnable runnable, double TIMEOUT)
   {
      Future<?> testResult = TIMEOUT_TEST_EXECUTOR.submit(runnable);
      try
      {
         testResult.get(Conversions.secondsToNanoseconds(TIMEOUT), TimeUnit.NANOSECONDS);
      }
      catch (ExecutionException | TimeoutException | InterruptedException exception)
      {
         if (exception instanceof  TimeoutException)
            fail("Failed to terminate before timeout");
         else
            throw new RuntimeException(exception);
      }
      finally
      {
         if (!testResult.isDone())
            testResult.cancel(true);
      }
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
         streamer.connectToCaller(localAddress, TEST_TIMEOUT);

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
      subscriber.connect(CALL_TIMEOUT);

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

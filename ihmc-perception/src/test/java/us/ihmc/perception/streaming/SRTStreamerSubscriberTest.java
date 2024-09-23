package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.Throttler;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;

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
      SRTVideoStreamer streamer = new SRTVideoStreamer();
      streamer.initialize(sampleImage.cols(), sampleImage.rows(), AV_PIX_FMT_BGR24);

      testTimeout(() -> streamer.sendFrame(sampleImage), TEST_TIMEOUT);                            // Try sending data to nowhere
      testTimeout(streamer::destroy, TEST_TIMEOUT);                                                // Try destroying

      // If it got here, the test passed!
   }

   @Test
   public void testSRTReceiverTimeouts()
   {
      double TIMEOUT = 5.0;
      InetSocketAddress streamerAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60001);

      SRTVideoReceiver receiver = new SRTVideoReceiver(AV_PIX_FMT_BGR24);
      testTimeout(() -> receiver.connect(streamerAddress, CALL_TIMEOUT), TIMEOUT);
      testTimeout(() -> receiver.getNextFrame(CALL_TIMEOUT), TIMEOUT);      // Try to get an image from nowhere
      testTimeout(receiver::destroy, TIMEOUT);                              // Try to destroy
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
   public void testSRTStreamerAndReceiver() throws InterruptedException
   {
      InetSocketAddress localAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60001);
      Throttler throttler = new Throttler();
      throttler.setFrequency(FPS);

      // Initialize the streamer
      SRTVideoStreamer streamer = new SRTVideoStreamer(localAddress);
      streamer.initialize(sampleImage.cols(), sampleImage.rows(), AV_PIX_FMT_BGR24);
      assertEquals(0, streamer.connectedCallerCount());

      // In a separate thread, wait for the receiver to connect
      Thread streamerConnectThread = ThreadTools.startAThread(() ->
      {
         // Send 2 seconds of video
         for (int i = 0; i < 2 * FPS; ++i)
         {
            throttler.waitAndRun();
            streamer.sendFrame(sampleImage);
         }
      }, "SRTStreamerTestConnection");

      // Initialize a receiver and connect to streamer
      SRTVideoReceiver receiver = new SRTVideoReceiver(AV_PIX_FMT_BGR24);
      assertFalse(receiver.isConnected());
      receiver.connect(localAddress, EXTRA_LONG_TIMEOUT);

      // Should be communicating now
      assertEquals(1, streamer.connectedCallerCount());
      assertTrue(receiver.isConnected());

      // Ensure we can receive an image
      Mat receivedImage = receiver.getNextFrame(0.5);
      assertNotNull(receivedImage);
      receivedImage.close();

      streamerConnectThread.join();

      streamer.destroy();
      receiver.destroy();

      // No communication
      assertEquals(0, streamer.connectedCallerCount());
      assertFalse(receiver.isConnected());
   }

   @Test
   public void testROS2SRTStreamerAndSubscriber() throws InterruptedException
   {
      InetSocketAddress localAddress = InetSocketAddress.createUnresolved("127.0.0.1", 60002);
      Throttler throttler = new Throttler();
      throttler.setFrequency(FPS);

      // ROS2 topic for the streamer and subscriber
      ROS2Topic<SRTStreamStatus> requestTopic = PerceptionAPI.STREAM_STATUS.withSuffix("ros2_srt_test");

      float depthDescretization = -1.0f;
      float fx = 500.0f;
      float fy = 400.0f;
      float cx = sampleImage.cols() / 2.0f;
      float cy = sampleImage.rows() / 2.0f;
      FramePoint3D testPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, 0.4, 0.5);
      FrameQuaternion testOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), 0.5, 0.4, 0.3);

      // Create an example raw image
      RawImage rawImage = new RawImage(0L,
                                       Instant.now(),
                                       depthDescretization,
                                       sampleImage,
                                       null,
                                       fx,
                                       fy,
                                       cx,
                                       cy,
                                       testPosition,
                                       testOrientation);

      // Create and initialize the streamer
      ROS2SRTVideoStreamer streamer = new ROS2SRTVideoStreamer(ROS2_NODE, requestTopic, localAddress);
      streamer.initialize(rawImage, AV_PIX_FMT_BGR24);

      // Create the subscriber
      ROS2SRTVideoSubscriber subscriber = new ROS2SRTVideoSubscriber(ROS2_HELPER, requestTopic, AV_PIX_FMT_BGR24);
      AtomicBoolean subscriberHasReceivedFrame = new AtomicBoolean(false);
      subscriber.addNewFrameConsumer(receivedImage ->
      {
         subscriberHasReceivedFrame.set(true);
         assertTrue(OpenCVTools.dimensionsMatch(receivedImage, sampleImage));
      });

      // No communication at this time
      assertEquals(0, streamer.connectedCallerCount());
      assertFalse(subscriber.isConnected());

      Thread sendThread = ThreadTools.startAThread(() ->
      {
         // Send 2 seconds of video
         for (int i = 0; i < 2 * FPS; ++i)
         {
            throttler.waitAndRun();
            streamer.sendFrame(rawImage);
         }
      }, "ROS2SRTStreamerSend");

      // Try subscribing
      subscriber.subscribe();
      MissingThreadTools.sleep(StreamingTools.CONNECTION_TIMEOUT);

      // Should be communicating now
      assertTrue(subscriber.isConnected());
      assertEquals(1, streamer.connectedCallerCount());

      // Ensure we can receive an image
      assertTrue(subscriberHasReceivedFrame.get());

      // Ensure we can receive other data
      assertEquals(depthDescretization, subscriber.getDepthDiscretization());
      assertEquals(fx, subscriber.getCameraIntrinsics().getFx());
      assertEquals(fy, subscriber.getCameraIntrinsics().getFy());
      assertEquals(cx, subscriber.getCameraIntrinsics().getCx());
      assertEquals(cy, subscriber.getCameraIntrinsics().getCy());

      FramePoint3D receivedPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), subscriber.getSensorTransformToWorld().getTranslation());
      FrameQuaternion receivedOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), subscriber.getSensorTransformToWorld().getRotation());
      EuclidCoreTestTools.assertEquals(testPosition, receivedPosition, 1E-5);
      EuclidCoreTestTools.assertEquals(testOrientation, receivedOrientation, 1E-5);

      sendThread.join();

      // Destroy everything
      streamer.destroy();
      subscriber.destroy();

      // No communication
      assertEquals(0, streamer.connectedCallerCount());
      assertFalse(subscriber.isConnected());
   }
}

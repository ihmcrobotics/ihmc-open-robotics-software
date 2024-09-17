package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
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

      SRTVideoReceiver subscriber = new SRTVideoReceiver(subscriberAddress, AV_PIX_FMT_BGR24);
      testTimeout(() -> subscriber.connect(CALL_TIMEOUT), TIMEOUT);
      testTimeout(() -> subscriber.waitForConnection(CALL_TIMEOUT), TIMEOUT); // Try waiting for a connection to nowhere
      testTimeout(() -> subscriber.getNextFrame(CALL_TIMEOUT), TIMEOUT);      // Try to get an image from nowhere
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
   public void testSRTStreamerAndReceiver() throws InterruptedException
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
      SRTVideoReceiver subscriber = new SRTVideoReceiver(localAddress, AV_PIX_FMT_BGR24);
      assertFalse(subscriber.isConnected());
      subscriber.connect(EXTRA_LONG_TIMEOUT);

      // Should be communicating now
      assertEquals(1, streamer.connectedCallerCount());
      assertTrue(subscriber.isConnected());

      // Ensure we can receive an image
      Mat receivedImage = subscriber.getNextFrame(0.5);
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
      ROS2IOTopicPair<SRTStreamMessage> requestTopic = new ROS2IOTopicPair<>(PerceptionAPI.STREAM_CONTROL.withSuffix("ros2_srt_test"));

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
      ROS2SRTVideoStreamer streamer = new ROS2SRTVideoStreamer(requestTopic);
      streamer.initialize(rawImage, FPS, AV_PIX_FMT_BGR24);

      // Create the subscriber
      ROS2SRTVideoSubscriber subscriber = new ROS2SRTVideoSubscriber(ROS2_HELPER, requestTopic, localAddress, AV_PIX_FMT_BGR24);
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

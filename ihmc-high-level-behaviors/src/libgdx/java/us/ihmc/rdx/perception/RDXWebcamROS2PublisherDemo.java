package us.ihmc.rdx.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.rdx.imgui.ImPlotIntegerPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

import java.time.Instant;

/**
 * Publishes webcam images to ROS 2 with best performance.
 */
public class RDXWebcamROS2PublisherDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ROS 2 Webcam Publisher");
   private final RDXPanel diagnosticPanel = new RDXPanel("Diagnostics", this::renderImGuiWidgets);
   private RDXOpenCVWebcamReader webcamReader;
   private BytePointer jpegImageBytePointer;
   private Mat yuv420Image;
   private final ImPlotStopwatchPlot encodeDurationPlot = new ImPlotStopwatchPlot("Encode duration");
   private final ImPlotFrequencyPlot publishFrequencyPlot = new ImPlotFrequencyPlot("Publish frequency");
   private final ImPlotIntegerPlot compressedBytesPlot = new ImPlotIntegerPlot("Compressed bytes");
   private final Stopwatch threadOneDuration = new Stopwatch();
   private final Stopwatch threadTwoDuration = new Stopwatch();
   private RealtimeROS2Node realtimeROS2Node;
   private ROS2PublisherBasics<BigVideoPacket> publisher;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private IntPointer compressionParameters;
   private final Runnable encodeAndPublish = this::encodeAndPublish;
   private final Object measurementSyncObject = new Object();
   private final Object encodeAndPublishMakeSureTheresOne = new Object();

   public RDXWebcamROS2PublisherDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            webcamReader = new RDXOpenCVWebcamReader();

            realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videopub");

            ROS2Topic<BigVideoPacket> bigVideoTestTopic = PerceptionAPI.BIG_VIDEO_TEST;
            publisher = realtimeROS2Node.createPublisher(BigVideoPacket.getPubSubType().get(), bigVideoTestTopic.getName(), ROS2QosProfile.BEST_EFFORT());
            realtimeROS2Node.spin();

            webcamReader.create();
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getImagePanel());

            yuv420Image = new Mat();
            jpegImageBytePointer = new BytePointer();

            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

            ThreadTools.startAsDaemon(() ->
            {
               while (true)
               {
                  // If encoding is running slower than reading, we want to wait a little so the read
                  // is freshest going into the encode and publish. We do this because we don't want
                  // to there to be more than one publish thread going
                  boolean shouldSleep;
                  double sleepTime;
                  synchronized (measurementSyncObject)
                  {
                     double averageThreadTwoDuration = threadTwoDuration.averageLap();
                     double averageThreadOneDuration = threadOneDuration.averageLap();
                     shouldSleep = !Double.isNaN(averageThreadTwoDuration) && !Double.isNaN(averageThreadOneDuration)
                                   && averageThreadTwoDuration > averageThreadOneDuration;
                     sleepTime = averageThreadTwoDuration - averageThreadOneDuration;

                     if (Double.isNaN(threadOneDuration.lap()))
                        threadOneDuration.reset();
                  }

                  if (shouldSleep)
                     ThreadTools.sleepSeconds(sleepTime);

                  // Convert colors are pretty fast. Encoding is slow, so let's do it in parallel.

                  webcamReader.readWebcamImage();

                  opencv_imgproc.cvtColor(webcamReader.getBGRImage(), yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);

                  synchronized (measurementSyncObject)
                  {
                     threadOneDuration.suspend();
                  }

                  ThreadTools.startAThread(encodeAndPublish, "EncodeAndPublish");
                  //                        encodeAndPublish.run();
               }
            }, "CameraRead");
         }

         @Override
         public void render()
         {
            webcamReader.updateOnUIThread();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            webcamReader.dispose();
            baseUI.dispose();
         }
      });
   }

   private void encodeAndPublish()
   {
      synchronized (encodeAndPublishMakeSureTheresOne)
      {
         synchronized (measurementSyncObject)
         {
            if (Double.isNaN(threadTwoDuration.lap()))
               threadTwoDuration.reset();
         }

         encodeDurationPlot.start();
         opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);
         encodeDurationPlot.stop();

         byte[] heapByteArrayData = new byte[jpegImageBytePointer.asBuffer().remaining()];
         jpegImageBytePointer.asBuffer().get(heapByteArrayData);
         videoPacket.getData().resetQuick();
         videoPacket.getData().add(heapByteArrayData);
         compressedBytesPlot.addValue(videoPacket.getData().size());
         Instant now = TimeTools.now();
         videoPacket.setAcquisitionTimeSecondsSinceEpoch(now.getEpochSecond());
         videoPacket.setAcquisitionTimeAdditionalNanos(now.getNano());
         videoPacket.setImageWidth(webcamReader.getImageWidth());
         videoPacket.setImageHeight(webcamReader.getImageHeight());
         publisher.publish(videoPacket);

         publishFrequencyPlot.ping();

         synchronized (measurementSyncObject)
         {
            threadTwoDuration.suspend();
         }
      }
   }

   private void renderImGuiWidgets()
   {
      webcamReader.renderImGuiWidgets();
      publishFrequencyPlot.renderImGuiWidgets();
      encodeDurationPlot.renderImGuiWidgets();
      compressedBytesPlot.renderImGuiWidgets();
   }

   public static void main(String[] args)
   {
      new RDXWebcamROS2PublisherDemo();
   }
}

package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.BigVideoPacket;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.Activator;

import java.io.IOException;
import java.time.Instant;

public class WebcamROS2PublisherDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "ROS 2 Webcam Publisher");
   private final ImGuiPanel diagnosticPanel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
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

   public WebcamROS2PublisherDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            webcamReader = new RDXOpenCVWebcamReader(nativesLoadedActivator);

            realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videopub");

            ROS2Topic<BigVideoPacket> bigVideoTestTopic = ROS2Tools.BIG_VIDEO_TEST;
            try
            {
               publisher = realtimeROS2Node.createPublisher(BigVideoPacket.getPubSubType().get(), bigVideoTestTopic.getName(), ROS2QosProfile.BEST_EFFORT());
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
            realtimeROS2Node.spin();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  webcamReader.create();
                  baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getVideoPanel());
                  baseUI.getLayoutManager().reloadLayout();

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

               webcamReader.updateOnUIThread();
            }

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
      if (nativesLoadedActivator.peek())
      {
         webcamReader.renderImGuiWidgets();
         publishFrequencyPlot.renderImGuiWidgets();
         encodeDurationPlot.renderImGuiWidgets();
         compressedBytesPlot.renderImGuiWidgets();
      }
   }

   public static void main(String[] args)
   {
      new WebcamROS2PublisherDemo();
   }
}

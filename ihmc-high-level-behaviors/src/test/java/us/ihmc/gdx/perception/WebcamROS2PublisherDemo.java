package us.ihmc.gdx.perception;

import controller_msgs.msg.dds.BigVideoPacket;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.ros2.RealtimeROS2Publisher;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.io.IOException;

public class WebcamROS2PublisherDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "ROS 2 Webcam Publisher");
   private final ImGuiPanel diagnosticPanel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
   private VideoCapture videoCapture;
   private int imageHeight = 1080;
   private int imageWidth = 1920;
   private double requestedFPS = 30.0;
   private double reportedFPS = 30.0;
   private String backendName = "";
   private Mat bgrImage;
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
   private ImPlotStopwatchPlot readPerformancePlot = new ImPlotStopwatchPlot("VideoCapture read(Mat) Duration");
   private ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Webcam OpenCV Read and ROS 2 Publish Frequency");
   private ImPlotIntegerPlot compressedBytesPlot = new ImPlotIntegerPlot("Compressed bytes");
   private RealtimeROS2Node realtimeROS2Node;
   private RealtimeROS2Publisher<BigVideoPacket> publisher;
   private BigVideoPacket videoPacket = new BigVideoPacket();
   private Throttler throttler = new Throttler();

   public WebcamROS2PublisherDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

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
                  videoCapture = new VideoCapture(0);

                  int reportedImageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  int reportedImageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

                  LogTools.info("Default resolution: {} x {}", reportedImageWidth, reportedImageHeight);
                  LogTools.info("Default fps: {}", reportedFPS);

                  backendName = BytedecoTools.stringFromByteBuffer(videoCapture.getBackendName());

                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, imageWidth);
                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, imageHeight);
                  videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
                  videoCapture.set(opencv_videoio.CAP_PROP_FPS, requestedFPS);
//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1280.0);
//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 720.0);

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);
                  LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));

                  bgrImage = new Mat();
                  Mat yuv420Image = new Mat();
                  BytePointer jpegImageBytePointer = new BytePointer();

                  swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Video", false);
                  baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

                  readPerformancePlot = new ImPlotStopwatchPlot("VideoCapture read(Mat)");

                  IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (true)
                     {
//                        throttler.waitAndRun(0.1);

                        readPerformancePlot.start();
                        boolean imageWasRead = videoCapture.read(bgrImage);
                        readPerformancePlot.stop();
                        readFrequencyPlot.ping();

                        if (!imageWasRead)
                        {
                           LogTools.error("Image was not read!");
                        }

                        opencv_imgproc.cvtColor(bgrImage, yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
                        opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);

                        byte[] heapByteArrayData = new byte[jpegImageBytePointer.asBuffer().remaining()];
                        jpegImageBytePointer.asBuffer().get(heapByteArrayData);
                        videoPacket.getData().reset();
                        videoPacket.getData().add(heapByteArrayData);
                        compressedBytesPlot.addValue(videoPacket.getData().size());
                        publisher.publish(videoPacket);

                        swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(data ->
                        {
                           data.updateOnImageUpdateThread(imageWidth, imageHeight);
                           opencv_imgproc.cvtColor(bgrImage, data.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
                        });
                     }
                  }, "CameraRead");
               }

               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
               {
                  data.updateOnUIThread(swapCVPanel.getVideoPanel());
               });
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            videoCapture.release();
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      if (nativesLoadedActivator.peek())
      {
         ImGui.text("Is open: " + videoCapture.isOpened());
         ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
         ImGui.text("Reported fps: " + reportedFPS);
         ImGui.text("Backend name: " + backendName);
         readPerformancePlot.renderImGuiWidgets();
         readFrequencyPlot.renderImGuiWidgets();
         compressedBytesPlot.renderImGuiWidgets();
      }
   }

   public static void main(String[] args)
   {
      new WebcamROS2PublisherDemo();
   }
}

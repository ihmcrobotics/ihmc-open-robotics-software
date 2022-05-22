package us.ihmc.gdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

public class WebcamROS2PublisherDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "ROS 2 Webcam Publisher");
   private final ImGuiPanel diagnosticPanel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
   private VideoCapture videoCapture;
   private int imageHeight = -1;
   private int imageWidth = -1;
   private String backendName = "";
//   private BytedecoImage rgbImage;
   private Mat rgbImage;
//   private Mat rgbaImage;
//   private GDXCVImagePanel cvImagePanel;
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
   private ImPlotStopwatchPlot readPerformancePlot = new ImPlotStopwatchPlot("VideoCapture read(Mat)");
   private ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("read Frequency");

   public WebcamROS2PublisherDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  videoCapture = new VideoCapture(0);

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

                  LogTools.info("Default resolution: {} x {}", imageWidth, imageHeight);

                  backendName = BytedecoTools.stringFromByteBuffer(videoCapture.getBackendName());

//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1920.0);
//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 1080.0);
//
//                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
//                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

//                  rgbImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
                  rgbImage = new Mat();
//                  rgbaImage = new Mat();

//                  cvImagePanel = new GDXCVImagePanel("Video", rgbImage);
//                  baseUI.getImGuiPanelManager().addPanel(cvImagePanel.getVideoPanel());

                  swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Video", false);
                  baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getVideoPanel());

                  readPerformancePlot = new ImPlotStopwatchPlot("VideoCapture read(Mat)");

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (true)
                     {
                        readPerformancePlot.start();
                        boolean imageWasRead = videoCapture.read(rgbImage);
                        readPerformancePlot.stop();
                        readFrequencyPlot.ping();

                        if (!imageWasRead)
                        {
                           LogTools.error("Image was not read!");
                        }

                        swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(data ->
                        {
                           data.updateOnImageUpdateThread(imageWidth, imageHeight);
                           opencv_imgproc.cvtColor(rgbImage, data.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
                        });
                     }
                  }, "CameraRead");

               }

//               readPerformancePlot.start();
//               boolean imageWasRead = videoCapture.read(rgbImage);
//               readPerformancePlot.stop();
//
//               if (imageWasRead)
//               {
//                  if (cvImagePanel == null)
//                  {
//                     rgbaImage = new Mat();
//
//                     opencv_imgproc.cvtColor(rgbImage, rgbaImage, opencv_imgproc.COLOR_RGB2RGBA, 0);
//
//                     cvImagePanel = new GDXCVImagePanel("Video", new BytedecoImage(rgbaImage));
//                     baseUI.getImGuiPanelManager().addPanel(cvImagePanel.getVideoPanel());
//                  }
//
//                  opencv_imgproc.cvtColor(rgbImage, rgbaImage, opencv_imgproc.COLOR_BGR2RGBA, 0);
//
//                  cvImagePanel.draw();
//               }

               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
               {
                  data.updateOnUIThread(swapCVPanel.getVideoPanel());
               });


               baseUI.renderBeforeOnScreenUI();
               baseUI.renderEnd();
            }
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
         ImGui.text("Backend name: " + backendName);
         readPerformancePlot.renderImGuiWidgets();
         readFrequencyPlot.renderImGuiWidgets();
      }
   }

   public static void main(String[] args)
   {
      new WebcamROS2PublisherDemo();
   }
}

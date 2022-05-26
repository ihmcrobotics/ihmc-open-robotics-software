package us.ihmc.gdx.logging;

import imgui.ImGui;
import org.bytedeco.ffmpeg.ffmpeg;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

import java.nio.ByteOrder;

public class GDXFFMPEGWebcamLoggingDemo
{
   private static final String WEBCAM_FILE = System.getProperty("webcam.file");
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread(opencv_core.class, ffmpeg.class);
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final boolean lossless = false;
   private final int framerate = 15;
   private final int bitrate = 1450000;
   private final FFMPEGLoggerDemoHelper ffmpegLoggerDemoHelper = new FFMPEGLoggerDemoHelper("FFMPEGWebcamLoggingDemo.webm",
                                                                                            avutil.AV_PIX_FMT_BGR24,
//                                                                                            avutil.AV_PIX_FMT_RGBA,
                                                                                            avutil.AV_PIX_FMT_YUV420P,
                                                                                            lossless,
                                                                                            framerate,
                                                                                            bitrate);
   private VideoCapture videoCapture;
   private BytedecoImage bgrImage;
   private int imageHeight = -1;
   private int imageWidth = -1;
   private double reportedFPS = -1;
   private String backendName = "";
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
   private ImPlotStopwatchPlot readPerformancePlot;
   private ImPlotFrequencyPlot readFrequencyPlot;

   public GDXFFMPEGWebcamLoggingDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  videoCapture = new VideoCapture(WEBCAM_FILE);

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

                  LogTools.info("Default resolution: {} x {}", imageWidth, imageHeight);
                  LogTools.info("Default fps: {}", reportedFPS);

                  backendName = BytedecoTools.stringFromByteBuffer(videoCapture.getBackendName());

//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1920.0);
//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 1080.0);
                  videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
                  videoCapture.set(opencv_videoio.CAP_PROP_FPS, framerate);
                  //                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1280.0);
                  //                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 720.0);

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);
                  LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));

                  bgrImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);

                  swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Video", false);
                  baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

                  readPerformancePlot = new ImPlotStopwatchPlot("VideoCapture read(Mat)");
                  readFrequencyPlot = new ImPlotFrequencyPlot("read Frequency");

                  ffmpegLoggerDemoHelper.create(imageWidth, imageHeight, () ->
                  {
                     swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(data ->
                     {
                        readPerformancePlot.start();
                        boolean imageWasRead = videoCapture.read(bgrImage.getBytedecoOpenCVMat());
                        readPerformancePlot.stop();
                        readFrequencyPlot.ping();

                        if (!imageWasRead)
                        {
                           LogTools.error("Image was not read!");
                        }

                        ffmpegLoggerDemoHelper.getLogger().put(bgrImage);

                        data.updateOnImageUpdateThread(imageWidth, imageHeight);
                        opencv_imgproc.cvtColor(bgrImage.getBytedecoOpenCVMat(), data.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
                     });
                  });

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
               {
                  data.updateOnUIThread(swapCVPanel.getVideoPanel());
               });
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());
            ffmpegLoggerDemoHelper.renderImGuiBasicInfo();

            if (nativesLoadedActivator.peek())
            {
               ImGui.text("Is open: " + videoCapture.isOpened());
               ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
               ImGui.text("Reported fps: " + reportedFPS);
               ImGui.text("Backend name: " + backendName);
               readPerformancePlot.renderImGuiWidgets();
               readFrequencyPlot.renderImGuiWidgets();

               ffmpegLoggerDemoHelper.renderImGuiNativesLoaded();
            }
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXFFMPEGWebcamLoggingDemo();
   }
}
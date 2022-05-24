package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

import java.io.File;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Date;

public class GDXFFMPEGWebcamLoggingDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private ImPlotFrequencyPlot loggerPutFrequencyPlot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private String fileName;
   private volatile boolean logging = false;
   private volatile boolean finalizing = false;
   private VideoCapture videoCapture;
   private Mat bgrImage;
   private int imageHeight = -1;
   private int imageWidth = -1;
   private double reportedFPS = -1;
   private String backendName = "";
   private final ImInt framerate = new ImInt(30);
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

            updateFileName();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  videoCapture = new VideoCapture("/dev/video2");

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

                  LogTools.info("Default resolution: {} x {}", imageWidth, imageHeight);
                  LogTools.info("Default fps: {}", reportedFPS);

                  backendName = BytedecoTools.stringFromByteBuffer(videoCapture.getBackendName());

//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1920.0);
//                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 1080.0);
                  videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
                  videoCapture.set(opencv_videoio.CAP_PROP_FPS, 30.0);
                  //                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1280.0);
                  //                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 720.0);

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);
                  LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));

                  bgrImage = new Mat();

                  swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Video", false);
                  baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

                  readPerformancePlot = new ImPlotStopwatchPlot("VideoCapture read(Mat)");
                  readFrequencyPlot = new ImPlotFrequencyPlot("read Frequency");

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (true)
                     {
                        readPerformancePlot.start();
                        boolean imageWasRead = videoCapture.read(bgrImage);
                        readPerformancePlot.stop();
                        readFrequencyPlot.ping();

                        if (!imageWasRead)
                        {
                           LogTools.error("Image was not read!");
                        }

                        swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(data ->
                        {
                           data.updateOnImageUpdateThread(imageWidth, imageHeight);
                           opencv_imgproc.cvtColor(bgrImage, data.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
                        });
                     }
                  }, "CameraRead");

                  loggerPutFrequencyPlot = new ImPlotFrequencyPlot("FFMPEGLogger put (Hz)");

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
            ImGui.text("File name: " + fileName);
            ImGui.inputInt(labels.get("framerate"), framerate, 1);

            if (nativesLoadedActivator.peek())
            {
               ImGui.text("Is open: " + videoCapture.isOpened());
               ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
               ImGui.text("Reported fps: " + reportedFPS);
               ImGui.text("Backend name: " + backendName);
               readPerformancePlot.renderImGuiWidgets();
               readFrequencyPlot.renderImGuiWidgets();

               if (!logging)
               {
                  if (!finalizing)
                  {
                     if (ImGui.button(labels.get("Start logging")))
                     {
                        updateFileName();
                        logging = true;
                        ThreadTools.startAThread(this::loggingThread, "FFMPEGLogging");
                     }
                  }
                  else
                  {
                     ImGui.text("Finalizing...");
                  }
               }
               else
               {
                  if (ImGui.button(labels.get("Stop logging")))
                  {
                     logging = false;
                  }
               }

               loggerPutFrequencyPlot.renderImGuiWidgets();
            }
         }

         private void updateFileName()
         {
            fileName = logDirectory + dateFormat.format(new Date()) + "_FFMPEGLoggingDemo.webm";
         }

         private void loggingThread()
         {
            boolean lossless = true;
            FFMPEGLogger logger = new FFMPEGLogger(imageWidth, imageHeight, lossless, framerate.get(), fileName);

            AVRational msBetweenFrames = new AVRational();
            msBetweenFrames.num(1);
            msBetweenFrames.den(framerate.get());

            finalizing = true;

            while (logging)
            {
               loggerPutFrequencyPlot.ping();

               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
               {
                  logger.put(data.getBytedecoImage());
               });

               // Using an AVRational helps ensure that we calculate fps the same way the logger does
               ThreadTools.sleep((int) (avutil.av_q2d(msBetweenFrames) * 1000));
            }

            ThreadTools.sleepSeconds(2.0);

            logger.close();
            finalizing = false;
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
package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.tools.thread.Activator;

public class RDXOpenCVWebcamReader
{
   private final ImGuiPanel panel = new ImGuiPanel("Webcam Reader", this::renderImGuiWidgets);
   private final Activator nativesLoadedActivator;
   private VideoCapture videoCapture;
   private int imageWidth = 1920;
   private int imageHeight = 1080;
   private double requestedFPS = 30.0;
   private double reportedFPS = 30.0;
   private String backendName = "";
   private Mat bgrImage;
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
   private final ImPlotStopwatchPlot readDurationPlot = new ImPlotStopwatchPlot("Read duration");
   private final ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Read frequency");

   public RDXOpenCVWebcamReader(Activator nativesLoadedActivator)
   {
      this.nativesLoadedActivator = nativesLoadedActivator;
   }

   public void create()
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
      // MJPG (Motion-JPEG) means to send each frame over USB as a JPEG compressed image
      // This is the only way webcams can stream at any decent framerate
      videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
      videoCapture.set(opencv_videoio.CAP_PROP_FPS, requestedFPS);

      imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);
      LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));

      bgrImage = new Mat();

      swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Webcam Monitor", false);
   }

   /**
    * This method works best if called asynchronously, as it runs slower
    * than UI framerates typically.
    *
    * However, it can also be called just before update() if in a pinch.
    */
   public void readWebcamImage()
   {
      readDurationPlot.start();
      boolean imageWasRead = videoCapture.read(bgrImage);
      readDurationPlot.stop();
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

   /**
    * This should be called on the render thread each frame. It will draw the
    * latest webcam image to the framebuffer.
    */
   public void update()
   {
      swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
      {
         data.updateOnUIThread(swapCVPanel.getVideoPanel());
      });
   }

   public void renderImGuiWidgets()
   {
      if (nativesLoadedActivator.peek())
      {
         ImGui.text("Is open: " + videoCapture.isOpened());
         ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
         ImGui.text("Reported fps: " + reportedFPS);
         ImGui.text("Backend name: " + backendName);
         readFrequencyPlot.renderImGuiWidgets();
         readDurationPlot.renderImGuiWidgets();
      }
   }

   public void dispose()
   {
      videoCapture.release();
   }

   public Mat getBGRImage()
   {
      return bgrImage;
   }

   public ImGuiPanel getStatisticsPanel()
   {
      return panel;
   }

   public ImGuiOpenCVSwapVideoPanel getSwapCVPanel()
   {
      return swapCVPanel;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }
}

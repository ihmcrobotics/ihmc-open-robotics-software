package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;
import us.ihmc.rdx.ui.graphics.RDXOpenCVGuidedSwapVideoPanel;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;

import java.util.function.Consumer;

/**
 * Reads and displays a webcam feed.
 */
public class RDXOpenCVWebcamReader
{
   private final RDXPanel panel = new RDXPanel("Webcam Reader", this::renderImGuiWidgets);
   private VideoCapture videoCapture;
   private int imageWidth = 1920;
   private int imageHeight = 1080;
   private double requestedFPS = 30.0;
   private double reportedFPS = 30.0;
   private String backendName = "";
   private Mat bgrImage;
   private RDXOpenCVGuidedSwapVideoPanel swapCVPanel;
   private final ImPlotStopwatchPlot readDurationPlot = new ImPlotStopwatchPlot("Read duration");
   private final ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Read frequency");
   private boolean imageWasRead = false;
   private long numberOfImagesRead = 0;
   private Consumer<RDXImagePanelTexture> monitorPanelUIThreadPreprocessor = null;

   public void create()
   {
      videoCapture = new VideoCapture(0);

      int reportedImageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int reportedImageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      LogTools.info("Default resolution: {} x {}", reportedImageWidth, reportedImageHeight);
      LogTools.info("Default fps: {}", reportedFPS);

      backendName = videoCapture.getBackendName().getString();

      // Set buffer size low to make sure not to get backed up
      videoCapture.set(opencv_videoio.CAP_PROP_BUFFERSIZE, 2);

      // Return an RGBA image so we don't have to do an extra conversion
      // These don't appear to work though
      //      videoCapture.set(opencv_videoio.CAP_PROP_FORMAT, opencv_core.CV_8UC4);
      //      videoCapture.set(opencv_videoio.CAP_PROP_CONVERT_RGB, 1);

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

      swapCVPanel = new RDXOpenCVGuidedSwapVideoPanel("Webcam Monitor", this::monitorPanelUpdateOnAsynchronousThread, this::monitorPanelUpdateOnUIThread);
      swapCVPanel.allocateInitialTextures(imageWidth, imageHeight);
   }

   /**
    * Allows the user to do some processing on the image after it is read
    * on the UI update thread. It's not ideal to do too much processing here,
    * just quick and easy stuff.
    */
   public void setMonitorPanelUIThreadPreprocessor(Consumer<RDXImagePanelTexture> monitorPanelUIThreadPreprocessor)
   {
      this.monitorPanelUIThreadPreprocessor = monitorPanelUIThreadPreprocessor;
   }

   /**
    * This method works best if called asynchronously, as it runs slower
    * than UI framerates typically.
    * <p>
    * However, it can also be called just before update() if in a pinch.
    */
   public void readWebcamImage()
   {
      readDurationPlot.start();
      swapCVPanel.updateOnAsynchronousThread();
      readDurationPlot.stop();
      readFrequencyPlot.ping();
   }

   private void monitorPanelUpdateOnAsynchronousThread(RDXImagePanelTexture texture)
   {
      imageWasRead = videoCapture.read(bgrImage);
      opencv_imgproc.cvtColor(bgrImage, texture.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
   }

   /**
    * This should be called on the render thread each frame. It will draw the
    * latest webcam image to the framebuffer.
    */
   public void updateOnUIThread()
   {
      swapCVPanel.updateOnUIThread();
   }

   private void monitorPanelUpdateOnUIThread(RDXImagePanelTexture texture)
   {
      if (imageWasRead)
      {
         imageWasRead = false;
         ++numberOfImagesRead;

         if (monitorPanelUIThreadPreprocessor != null && texture.getRGBA8Image() != null)
            monitorPanelUIThreadPreprocessor.accept(texture);

         texture.updateTextureAndDraw(swapCVPanel.getImagePanel());
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Is open: " + videoCapture.isOpened());
      ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.text("Reported fps: " + reportedFPS);
      ImGui.text("Backend name: " + backendName);
      ImGui.text("Number of images read: " + numberOfImagesRead);
      readFrequencyPlot.renderImGuiWidgets();
      readDurationPlot.renderImGuiWidgets();
   }

   public void dispose()
   {
      videoCapture.release();
   }

   public boolean getImageWasRead()
   {
      return imageWasRead;
   }

   public Mat getBGRImage()
   {
      return bgrImage;
   }

   public RDXPanel getStatisticsPanel()
   {
      return panel;
   }

   public RDXOpenCVGuidedSwapVideoPanel getSwapCVPanel()
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

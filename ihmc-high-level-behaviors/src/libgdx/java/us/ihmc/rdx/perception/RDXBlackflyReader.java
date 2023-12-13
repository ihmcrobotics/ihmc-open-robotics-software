package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;
import us.ihmc.rdx.ui.graphics.RDXOpenCVSwapVideoPanel;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;

import java.util.function.Consumer;

/**
 * This class handles reading a locally plugged in Blackfly camera and displaying the live
 * result with good performance.
 */
public class RDXBlackflyReader
{
   private final RDXPanel panel = new RDXPanel("Blackfly Reader", this::renderImGuiWidgets);
   private final String serialNumber;
   private volatile int imageWidth = -1;
   private volatile int imageHeight = -1;
   private SpinnakerBlackflyManager spinnakerBlackflyManager;
   private SpinnakerBlackfly blackfly;
   private spinImage spinImage;
   private BytePointer spinImageDataPointer;
   private Mat blackflySourceMat;
   private RDXOpenCVSwapVideoPanel swapImagePanel;
   private final ImPlotStopwatchPlot readDurationPlot = new ImPlotStopwatchPlot("Read duration");
   private final ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Read frequency");
   private boolean imageWasRead = false;
   private long numberOfImagesRead = 0;
   private Consumer<RDXImagePanelTexture> monitorPanelUIThreadPreprocessor = null;

   public RDXBlackflyReader(String serialNumber)
   {
      this.serialNumber = serialNumber;
   }

   public void create()
   {
      spinnakerBlackflyManager = new SpinnakerBlackflyManager();
      blackfly = spinnakerBlackflyManager.createSpinnakerBlackfly(serialNumber);
      spinImage = new spinImage();
      swapImagePanel = new RDXOpenCVSwapVideoPanel("Blackfly Monitor");
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
   public void readBlackflyImage()
   {
      readDurationPlot.start();

      imageWasRead = blackfly.getNextImage(spinImage);

      if (imageWasRead)
      {
         if (spinImageDataPointer == null)
         {
            imageWidth = blackfly.getWidth(spinImage);
            imageHeight = blackfly.getHeight(spinImage);
            spinImageDataPointer = new BytePointer(imageWidth * imageHeight * 3); // 8 bits per pixel
            blackflySourceMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8U);
            swapImagePanel.allocateInitialTextures(imageWidth, imageHeight);
         }

         Spinnaker_C.spinImageGetData(spinImage, spinImageDataPointer);
         blackflySourceMat.data(spinImageDataPointer);

         // Here we use COLOR_BayerBG2RGBA opencv conversion. The Blackfly cameras are set to use BayerRG pixel format.
         // But, for some reason, it's actually BayerBG. Changing to COLOR_BayerRG2RGBA will result in the wrong colors.
         opencv_imgproc.cvtColor(blackflySourceMat, swapImagePanel.getAsynchronousThreadData().getRGBA8Mat(), opencv_imgproc.COLOR_BayerBG2RGBA);
         swapImagePanel.swap();

         Spinnaker_C.spinImageRelease(spinImage);
      }

      readDurationPlot.stop();
      readFrequencyPlot.ping();
   }

   /**
    * This should be called on the render thread each frame. It will draw the
    * latest image to the framebuffer.
    */
   public void updateOnUIThread()
   {
      if (imageWasRead)
      {
         imageWasRead = false;
         ++numberOfImagesRead;

         synchronized (swapImagePanel.getSyncObject())
         {
            RDXImagePanelTexture texture = swapImagePanel.getUIThreadData();

            if (monitorPanelUIThreadPreprocessor != null && texture.getRGBA8Image() != null)
               monitorPanelUIThreadPreprocessor.accept(texture);

            texture.updateTextureAndDraw(swapImagePanel.getImagePanel());
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Serial number: " + serialNumber);
      ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.text("Number of images read: " + numberOfImagesRead);
      readFrequencyPlot.renderImGuiWidgets();
      readDurationPlot.renderImGuiWidgets();
   }

   public void dispose()
   {
      ThreadTools.sleep(250);
      blackfly.stopAcquiringImages();
      ThreadTools.sleep(100);
      spinnakerBlackflyManager.destroy();
   }

   public boolean getImageWasRead()
   {
      return imageWasRead;
   }

   public RDXPanel getStatisticsPanel()
   {
      return panel;
   }

   public RDXOpenCVSwapVideoPanel getSwapImagePanel()
   {
      return swapImagePanel;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public Mat getBayerBGImage()
   {
      return blackflySourceMat;
   }
}

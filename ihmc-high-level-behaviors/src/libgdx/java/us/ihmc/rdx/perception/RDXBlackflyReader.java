package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerSystemManager;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.tools.thread.Activator;

import java.util.function.Consumer;

public class RDXBlackflyReader
{
   private final ImGuiPanel panel = new ImGuiPanel("Blackfly Reader", this::renderImGuiWidgets);
   private final Activator nativesLoadedActivator;
   private String serialNumber;
   private volatile long imageWidth = -1;
   private volatile long imageHeight = -1;
   private SpinnakerSystemManager spinnakerSystemManager;
   private SpinnakerBlackfly blackfly;
   private spinImage spinImage;
   private BytePointer spinImageDataPointer;
   private Mat blackflySourceMat;
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
   private final ImPlotStopwatchPlot readDurationPlot = new ImPlotStopwatchPlot("Read duration");
   private final ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Read frequency");
   private final Consumer<ImGuiOpenCVSwapVideoPanelData> accessOnLowPriorityThread = this::accessOnLowPriorityThread;
   private final Consumer<ImGuiOpenCVSwapVideoPanelData> accessOnHighPriorityThread = this::accessOnHighPriorityThread;
   private boolean imageWasRead = false;
   private long numberOfImagesRead = 0;

   public RDXBlackflyReader(Activator nativesLoadedActivator, String serialNumber)
   {
      this.nativesLoadedActivator = nativesLoadedActivator;
      this.serialNumber = serialNumber;
   }

   public void create()
   {
      spinnakerSystemManager = new SpinnakerSystemManager();
      blackfly = spinnakerSystemManager.createBlackfly(serialNumber);

      spinImage = new spinImage();

      blackfly.setAcquisitionMode(Spinnaker_C.spinAcquisitionModeEnums.AcquisitionMode_Continuous);
      blackfly.setPixelFormat(Spinnaker_C.spinPixelFormatEnums.PixelFormat_RGB8);
      blackfly.startAcquiringImages();

      swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Blackfly Monitor", false);
   }

   /**
    * This method works best if called asynchronously, as it runs slower
    * than UI framerates typically.
    *
    * However, it can also be called just before update() if in a pinch.
    */
   public void readBlackflyImage()
   {
      readDurationPlot.start();
      swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(accessOnLowPriorityThread);
      readDurationPlot.stop();
      readFrequencyPlot.ping();
   }

   private void accessOnLowPriorityThread(ImGuiOpenCVSwapVideoPanelData data)
   {
      imageWasRead = blackfly.getNextImage(spinImage);

      if (imageWasRead)
      {
         if (spinImageDataPointer == null)
         {
            imageWidth = blackfly.getWidth(spinImage);
            imageHeight = blackfly.getHeight(spinImage);
            spinImageDataPointer = new BytePointer(imageWidth * imageHeight * 3); // RGB8
            blackflySourceMat = new Mat((int) imageHeight, (int) imageWidth, opencv_core.CV_8UC3);
            swapCVPanel.getDataSwapReferenceManager().initializeBoth(data2 -> data2.updateOnImageUpdateThread((int) imageWidth, (int) imageHeight));
         }

         Spinnaker_C.spinImageGetData(spinImage, spinImageDataPointer);
         blackflySourceMat.data(spinImageDataPointer);

         opencv_imgproc.cvtColor(blackflySourceMat, data.getRGBA8Mat(), opencv_imgproc.COLOR_RGB2RGBA, 0);
         Spinnaker_C.spinImageRelease(spinImage);
      }
   }

   /**
    * This should be called on the render thread each frame. It will draw the
    * latest image to the framebuffer.
    */
   public void update()
   {
      swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(accessOnHighPriorityThread);
   }

   /**
    * For advanced use, allow the user to call more stuff on the high priority thread.
    * You would call this instead of update()
    */
   public void accessOnHighPriorityThread(ImGuiOpenCVSwapVideoPanelData data)
   {
      if (imageWasRead)
      {
         imageWasRead = false;
         ++numberOfImagesRead;
         data.updateOnUIThread(swapCVPanel.getVideoPanel());
      }
   }

   public void renderImGuiWidgets()
   {
      if (nativesLoadedActivator.peek())
      {
         ImGui.text("Serial number: " + serialNumber);
         ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
         ImGui.text("Number of images read: " + numberOfImagesRead);
         readFrequencyPlot.renderImGuiWidgets();
         readDurationPlot.renderImGuiWidgets();
      }
   }

   public void dispose()
   {
      blackfly.stopAcquiringImages();
      spinnakerSystemManager.destroy();
   }

   public boolean getImageWasRead()
   {
      return imageWasRead;
   }

   public ImGuiPanel getStatisticsPanel()
   {
      return panel;
   }

   public ImGuiOpenCVSwapVideoPanel getSwapCVPanel()
   {
      return swapCVPanel;
   }

   public long getImageHeight()
   {
      return imageHeight;
   }

   public long getImageWidth()
   {
      return imageWidth;
   }
}

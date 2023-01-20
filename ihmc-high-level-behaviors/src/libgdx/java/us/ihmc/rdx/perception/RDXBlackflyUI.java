package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import org.bytedeco.spinnaker.Spinnaker_C.spinImageProcessor;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.spinnaker.SpinnakerTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerSystemManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class RDXBlackflyUI
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private SpinnakerBlackfly blackfly;
   private AtomicBoolean doImageAcquisition;
   private Thread imageAcquisitionService;
   private spinImageProcessor spinImageProcessor;
   private AtomicReference<spinImage> currentUnprocessedImage;
   private spinImage previousImage = null;
   private spinImage currentImage = null;
   private SpinnakerSystemManager spinnakerSystemManager;
   private RDXCVImagePanel imagePanel;
   private BytePointer imageData;
   private FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImPlotStopwatchPlot processDurationPlot = new ImPlotStopwatchPlot("Process duration");

   public RDXBlackflyUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Blackfly", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  spinnakerSystemManager = new SpinnakerSystemManager();
                  blackfly = spinnakerSystemManager.createBlackfly(BLACKFLY_SERIAL_NUMBER);
                  blackfly.setAcquisitionMode(Spinnaker_C.spinAcquisitionModeEnums.AcquisitionMode_Continuous);
                  blackfly.setPixelFormat(Spinnaker_C.spinPixelFormatEnums.PixelFormat_RGB8);
                  blackfly.startAcquiringImages();
                  // Image acquisition needs to run on a different thread so that the whole program doesn't need to wait for new images
                  doImageAcquisition = new AtomicBoolean(true);
                  spinImageProcessor = new spinImageProcessor();
                  SpinnakerTools.assertNoError(Spinnaker_C.spinImageProcessorCreate(spinImageProcessor), "Error creating image processor");
                  currentUnprocessedImage = new AtomicReference<>(null);
                  imageAcquisitionService = ThreadTools.startAThread(() ->
                  {
                     while (doImageAcquisition.get())
                     {
                        spinImage spinImage = new spinImage();

                        if (!blackfly.getNextImage(spinImage))
                        {
                           Spinnaker_C.spinImageRelease(spinImage);
                           continue;
                        }

                        spinImage oldImage = currentUnprocessedImage.get();
                        currentUnprocessedImage.set(spinImage);
                        Spinnaker_C.spinImageRelease(oldImage);
                     }
                  }, "Blackfly " + BLACKFLY_SERIAL_NUMBER + " Image Acquisition");
               }

               if (getLatestBlackflyImage())
               {
                  if (imagePanel == null)
                  {
                     int imageWidth = blackfly.getWidth(currentImage);
                     int imageHeight = blackfly.getHeight(currentImage);
                     LogTools.info("Blackfly {} resolution detected: {}x{}", BLACKFLY_SERIAL_NUMBER, imageWidth, imageHeight);
                     imagePanel = new RDXCVImagePanel("Blackfly Image", imageWidth, imageHeight);
                     baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());

                     imageData = new BytePointer((long) imageWidth * imageHeight * 4); // Each pixel has 4 bytes of data, hence * 4

                     baseUI.getPerspectiveManager().reloadPerspective();
                  }

                  frameReadFrequency.ping();
                  imagePanel.getBytedecoImage().rewind();

                  processDurationPlot.start();
                  blackfly.setBytedecoPointerToSpinImageData(currentImage, imageData);
                  imagePanel.updateDataAddress(imageData.address());
                  processDurationPlot.stop();
               }

               if (imagePanel != null)
                  imagePanel.draw();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private boolean getLatestBlackflyImage()
         {
            if (currentUnprocessedImage.get() == previousImage || currentUnprocessedImage.get() == null)
               return false;

            spinImage spinImage = new spinImage();
            Spinnaker_C.spinImageCreateEmpty(spinImage);
            Spinnaker_C.spinImageProcessorConvert(spinImageProcessor,
                                                  currentUnprocessedImage.get(),
                                                  spinImage,
                                                  Spinnaker_C.spinPixelFormatEnums.PixelFormat_RGBa8);

            spinImage oldImage = currentImage;

            previousImage = currentUnprocessedImage.get();
            currentImage = spinImage;

            if (oldImage != null)
               Spinnaker_C.spinImageDestroy(oldImage);

            return true;
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (imagePanel != null)
            {
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());

               processDurationPlot.renderImGuiWidgets();

               ImGui.text("R G B A:");

               imagePanel.getBytedecoImage().rewind();
               for (int i = 0; i < 5; i++)
               {
                  printBytes(imagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                             imagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                             imagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                             imagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
               }
            }
         }

         private void printBytes(byte byte0, byte byte1, byte byte2, byte byte3)
         {
            printInts(Byte.toUnsignedInt(byte0),
                      Byte.toUnsignedInt(byte1),
                      Byte.toUnsignedInt(byte2),
                      Byte.toUnsignedInt(byte3));
         }

         private void printInts(int int0, int int1, int int2, int int3)
         {
            ImGui.text(int0 + " " + int1 + " " + int2 + " " + int3);
         }

         @Override
         public void dispose()
         {
            doImageAcquisition.set(false);
            ExceptionTools.handle(() -> imageAcquisitionService.wait(), DefaultExceptionHandler.PRINT_MESSAGE);
            Spinnaker_C.spinImageRelease(currentUnprocessedImage.get());

            spinnakerSystemManager.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXBlackflyUI();
   }
}

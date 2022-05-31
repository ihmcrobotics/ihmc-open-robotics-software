package us.ihmc.gdx.perception;

import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerSystemManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class GDXBlackflyUI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private SpinnakerBlackfly blackfly;
   private AtomicBoolean doImageAcquisition;
   private Thread imageAcquisitionService;
   private AtomicReference<spinImage> currentUnprocessedImage;
   private spinImage previousImage = null;
   private spinImage currentImage = null;
   private SpinnakerSystemManager spinnakerSystemManager;
   private GDXCVImagePanel imagePanel;
   private BytePointer imageData;
   private FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImPlotStopwatchPlot processDurationPlot = new ImPlotStopwatchPlot("Process duration");
   private String serialNumber;

   public GDXBlackflyUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
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
                  serialNumber = "17403057";
                  blackfly = spinnakerSystemManager.createBlackfly(serialNumber);
                  blackfly.setAcquisitionMode(Spinnaker_C.spinAcquisitionModeEnums.AcquisitionMode_Continuous);
                  blackfly.setPixelFormat(Spinnaker_C.spinPixelFormatEnums.PixelFormat_RGB8);
                  blackfly.startAcquiringImages();
                  // Image acquisition needs to run on a different thread so that the whole program doesn't need to wait for new images
                  doImageAcquisition = new AtomicBoolean(true);
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
                  }, "Blackfly " + serialNumber + " Image Acquisition");
               }

               if (getLatestBlackflyImage())
               {
                  if (imagePanel == null)
                  {
                     int imageWidth = blackfly.getWidth(currentImage);
                     int imageHeight = blackfly.getHeight(currentImage);
                     LogTools.info("Blackfly {} resolution detected: {}x{}", serialNumber, imageWidth, imageHeight);
                     imagePanel = new GDXCVImagePanel("Blackfly Image", imageWidth, imageHeight);
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
            Spinnaker_C.spinImageConvert(currentUnprocessedImage.get(), Spinnaker_C.spinPixelFormatEnums.PixelFormat_RGBa8, spinImage);

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
      new GDXBlackflyUI();
   }
}

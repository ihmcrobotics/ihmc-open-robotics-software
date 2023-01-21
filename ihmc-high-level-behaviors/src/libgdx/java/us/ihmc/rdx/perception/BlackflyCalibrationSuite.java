package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.opencv_core.Point3fVectorVector;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.logging.HDF5ImageLogging;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.tools.thread.Activator;

import java.util.function.Consumer;

public class BlackflyCalibrationSuite
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/libgdx/resources",
                                                  "Blackfly Calibration Suite");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXBlackflyReader blackflyReader;
   private CalibrationPatternDetection calibrationPatternDetection;
   private HDF5ImageLogging hdf5ImageLogging;
   private volatile boolean running = true;
   private final Consumer<ImGuiOpenCVSwapVideoPanelData> accessOnHighPriorityThread = this::accessOnHighPriorityThread;
   private Point3fVectorVector objectPoints;

   public BlackflyCalibrationSuite()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            blackflyReader = new RDXBlackflyReader(nativesLoadedActivator, BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());

            baseUI.getImGuiPanelManager().addPanel("Calibration", BlackflyCalibrationSuite.this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  blackflyReader.create();
                  baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapCVPanel().getVideoPanel());

                  calibrationPatternDetection = new CalibrationPatternDetection();
                  baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetection.getPanel());

                  baseUI.getPerspectiveManager().reloadPerspective();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        blackflyReader.readBlackflyImage();
                        calibrationPatternDetection.copyRGBImage(blackflyReader.getRGBImage());

                        if (hdf5ImageLogging != null)
                           hdf5ImageLogging.copyRGBImage(blackflyReader.getRGBImage());
                     }
                  }, "CameraRead");
               }

               calibrationPatternDetection.update();
               blackflyReader.getSwapCVPanel().getDataSwapReferenceManager().accessOnHighPriorityThread(accessOnHighPriorityThread);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            running = false;
            blackflyReader.dispose();
            baseUI.dispose();
         }
      });
   }

   private void accessOnHighPriorityThread(ImGuiOpenCVSwapVideoPanelData data)
   {
      if (data.getRGBA8Image() != null)
      {
         if (blackflyReader.getImageWasRead())
         {
            if (hdf5ImageLogging == null)
            {
               hdf5ImageLogging = new HDF5ImageLogging(nativesLoadedActivator, (int) blackflyReader.getImageWidth(), (int) blackflyReader.getImageHeight());
               baseUI.getImGuiPanelManager().addPanel(hdf5ImageLogging.getPanel());
               baseUI.getPerspectiveManager().reloadPerspective();
            }

            calibrationPatternDetection.drawCornersOrCenters(data.getRGBA8Mat());
         }

         blackflyReader.accessOnHighPriorityThread(data);
      }
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("Start recording data set")))
      {

      }

      if (ImGui.button(labels.get("Capture calibration image")))
      {

      }

      if (ImGui.button(labels.get("Calibrate")))
      {
         ThreadTools.startAsDaemon(this::calibrate, "Calibration");
      }
   }

   private void calibrate()
   {
      objectPoints = new Point3fVectorVector();


//      opencv_calib3d.calibrate();
   }

   public static void main(String[] args)
   {
      new BlackflyCalibrationSuite();
   }
}

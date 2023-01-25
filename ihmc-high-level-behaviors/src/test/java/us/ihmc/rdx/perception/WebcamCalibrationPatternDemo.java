package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.tools.thread.Activator;

import java.util.function.Consumer;

public class WebcamCalibrationPatternDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Webcam Calibration Pattern Demo");
   private RDXOpenCVWebcamReader webcamReader;
   private CalibrationPatternDetectionUI calibrationPatternDetectionUI;
   private volatile boolean running = true;
   private final Consumer<ImGuiOpenCVSwapVideoPanelData> accessOnHighPriorityThread = this::accessOnHighPriorityThread;

   public WebcamCalibrationPatternDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            webcamReader = new RDXOpenCVWebcamReader(nativesLoadedActivator);
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getStatisticsPanel());
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  webcamReader.create();
                  baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getVideoPanel());

                  calibrationPatternDetectionUI = new CalibrationPatternDetectionUI();
                  baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetectionUI.getPanel());
                  baseUI.getLayoutManager().reloadLayout();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        webcamReader.readWebcamImage();
                        calibrationPatternDetectionUI.copyBGRImage(webcamReader.getBGRImage());
                     }
                  }, "CameraRead");
               }

               calibrationPatternDetectionUI.update();
               webcamReader.getSwapCVPanel().getDataSwapReferenceManager().accessOnHighPriorityThread(accessOnHighPriorityThread);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            running = false;
            webcamReader.dispose();
            baseUI.dispose();
         }
      });
   }

   private void accessOnHighPriorityThread(ImGuiOpenCVSwapVideoPanelData data)
   {
      if (webcamReader.getImageWasRead())
      {
         calibrationPatternDetectionUI.drawCornersOrCenters(data.getRGBA8Mat());
      }

      webcamReader.accessOnHighPriorityThread(data);
   }

   public static void main(String[] args)
   {
      new WebcamCalibrationPatternDemo();
   }
}

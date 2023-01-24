package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.logging.HDF5ImageLogging;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.Activator;

public class WebcamHDF5LoggingDemo
{

   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Webcam HDF5 Logging Demo");
   private HDF5ImageLogging hdf5ImageLogging;
   private RDXOpenCVWebcamReader webcamReader;
   private volatile boolean running = true;

   public WebcamHDF5LoggingDemo()
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

                  hdf5ImageLogging = new HDF5ImageLogging(nativesLoadedActivator, webcamReader.getImageWidth(), webcamReader.getImageHeight());
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageLogging.getPanel());
                  baseUI.getLayoutManager().reloadLayout();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        webcamReader.readWebcamImage();
                        hdf5ImageLogging.copyBGRImage(webcamReader.getBGRImage());
                     }
                  }, "CameraRead");
               }

               webcamReader.update();
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

   public static void main(String[] args)
   {
      new WebcamHDF5LoggingDemo();
   }
}

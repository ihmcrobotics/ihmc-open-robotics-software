package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.logging.RDXHDF5ImageLoggingUI;
import us.ihmc.rdx.ui.RDXBaseUI;

/**
 * Log webcam images to HDF5.
 */
public class RDXWebcamHDF5LoggingDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Webcam HDF5 Logging Demo");
   private RDXHDF5ImageLoggingUI hdf5ImageLoggingUI;
   private RDXOpenCVWebcamReader webcamReader;
   private volatile boolean running = true;

   public RDXWebcamHDF5LoggingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            webcamReader = new RDXOpenCVWebcamReader();
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getStatisticsPanel());

            webcamReader.create();
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getImagePanel());

            hdf5ImageLoggingUI = new RDXHDF5ImageLoggingUI(webcamReader.getImageWidth(), webcamReader.getImageHeight());
            baseUI.getImGuiPanelManager().addPanel(hdf5ImageLoggingUI.getPanel());

            ThreadTools.startAsDaemon(() ->
            {
               while (running)
               {
                  webcamReader.readWebcamImage();
                  hdf5ImageLoggingUI.copyBGRImage(webcamReader.getBGRImage());
               }
            }, "CameraRead");
         }

         @Override
         public void render()
         {
            webcamReader.updateOnUIThread();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            running = false;
            webcamReader.dispose();
            hdf5ImageLoggingUI.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXWebcamHDF5LoggingDemo();
   }
}

package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;

/**
 * Renders a webcam with good performance.
 */
public class RDXWebcamDisplayDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Webcam Display Demo");
   private RDXOpenCVWebcamReader webcamReader;
   private volatile boolean running = true;

   public RDXWebcamDisplayDemo()
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

            ThreadTools.startAsDaemon(() ->
                                      {
                                         while (running)
                                         {
                                            webcamReader.readWebcamImage();
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
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXWebcamDisplayDemo();
   }
}

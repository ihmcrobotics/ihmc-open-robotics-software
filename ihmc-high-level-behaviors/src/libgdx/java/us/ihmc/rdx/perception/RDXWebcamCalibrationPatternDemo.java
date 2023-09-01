package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;

/**
 * Interactive calibration pattern detection with a webcam.
 */
public class RDXWebcamCalibrationPatternDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Webcam Calibration Pattern Demo");
   private RDXOpenCVWebcamReader webcamReader;
   private RDXCalibrationPatternDetectionUI calibrationPatternDetectionUI;
   private volatile boolean running = true;

   public RDXWebcamCalibrationPatternDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            webcamReader = new RDXOpenCVWebcamReader();
            webcamReader.setMonitorPanelUIThreadPreprocessor(this::monitorPanelUpdateOnUIThread);
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getStatisticsPanel());

            webcamReader.create();
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getImagePanel());

            calibrationPatternDetectionUI = new RDXCalibrationPatternDetectionUI();
            baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetectionUI.getPanel());

            ThreadTools.startAsDaemon(() ->
            {
               while (running)
               {
                  webcamReader.readWebcamImage();
                  calibrationPatternDetectionUI.copyInSourceBGRImage(webcamReader.getBGRImage());
               }
            }, "CameraRead");
         }

         @Override
         public void render()
         {
            calibrationPatternDetectionUI.update();
            webcamReader.updateOnUIThread();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void monitorPanelUpdateOnUIThread(RDXImagePanelTexture texture)
         {
            calibrationPatternDetectionUI.drawCornersOrCenters(texture.getRGBA8Mat());
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
      new RDXWebcamCalibrationPatternDemo();
   }
}

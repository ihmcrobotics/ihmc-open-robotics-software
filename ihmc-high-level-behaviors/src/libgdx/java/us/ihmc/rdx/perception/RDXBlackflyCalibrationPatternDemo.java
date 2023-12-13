package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;

/**
 * This app view the live feed of a locally plugged in Blackfly and visualizes
 * calibration pattern detection on it.
 */
public class RDXBlackflyCalibrationPatternDemo
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI("Blackfly Calibration Pattern Demo");
   private RDXBlackflyReader blackflyReader;
   private RDXCalibrationPatternDetectionUI calibrationPatternDetectionUI;
   private volatile boolean running = true;

   public RDXBlackflyCalibrationPatternDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            blackflyReader = new RDXBlackflyReader(BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());

            blackflyReader.create();
            blackflyReader.setMonitorPanelUIThreadPreprocessor(this::monitorUIThreadPreprocessor);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapImagePanel().getImagePanel());

            calibrationPatternDetectionUI = new RDXCalibrationPatternDetectionUI();
            baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetectionUI.getPanel());

            ThreadTools.startAsDaemon(() ->
            {
               while (running)
               {
                  blackflyReader.readBlackflyImage();
                  calibrationPatternDetectionUI.copyBayerBGImage(blackflyReader.getBayerBGImage());
               }
            }, "CameraRead");
         }

         @Override
         public void render()
         {
            calibrationPatternDetectionUI.update();
            blackflyReader.updateOnUIThread();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void monitorUIThreadPreprocessor(RDXImagePanelTexture texture)
         {
            calibrationPatternDetectionUI.drawCornersOrCenters(texture.getRGBA8Mat());
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

   public static void main(String[] args)
   {
      new RDXBlackflyCalibrationPatternDemo();
   }
}

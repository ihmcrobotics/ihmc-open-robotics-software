package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;

/**
 * This application shows the live feed of a locally plugged in Blackfly camera.
 */
public class RDXBlackflyDisplayDemo
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI("Blackfly Display Demo");
   private RDXBlackflyReader blackflyReader;
   private volatile boolean running = true;

   public RDXBlackflyDisplayDemo()
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
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapImagePanel().getImagePanel());

            ThreadTools.startAsDaemon(() ->
            {
               while (running)
               {
                  blackflyReader.readBlackflyImage();
               }
            }, "CameraRead");
         }

         @Override
         public void render()
         {
            blackflyReader.updateOnUIThread();
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

   public static void main(String[] args)
   {
      new RDXBlackflyDisplayDemo();
   }
}

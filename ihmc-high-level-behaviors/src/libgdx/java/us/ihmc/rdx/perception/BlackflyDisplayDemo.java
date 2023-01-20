package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.Activator;

public class BlackflyDisplayDemo
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/libgdx/resources",
                                                  "Blackfly Display Demo");
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
   private RDXBlackflyReader blackflyReader;
   private volatile boolean running = true;

   public BlackflyDisplayDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            blackflyReader = new RDXBlackflyReader(nativesLoadedActivator, BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());
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
                  baseUI.getPerspectiveManager().reloadPerspective();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        blackflyReader.readBlackflyImage();
                     }
                  }, "CameraRead");
               }

               blackflyReader.update();
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

   public static void main(String[] args)
   {
      new BlackflyDisplayDemo();
   }
}

package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

public class RDXPlanarRegionLoggingDevelopmentUI
{

   private final String WINDOW_NAME = "Planar Region Logging Development UI";

   private final RDXBaseUI baseUI;

   private RDXPlanarRegionLoggingPanel panel;

   private final Timer timer;
   private final Random rand;

   public RDXPlanarRegionLoggingDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new RDXBaseUI(WINDOW_NAME);
      timer = new Timer();
      rand = new Random();
   }

   public void launch()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            panel = new RDXPlanarRegionLoggingPanel();
            panel.create();
            baseUI.getImGuiPanelManager().addPanel(panel.getWindowName(), panel::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(panel);

            timer.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  PlanarRegionsList regions = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(rand, 5, 5, 5, 5);
                  panel.update(System.nanoTime(), regions);
               }
            }, 50, 100);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            panel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      RDXPlanarRegionLoggingDevelopmentUI ui = new RDXPlanarRegionLoggingDevelopmentUI();
      ui.launch();
   }
}

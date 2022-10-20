package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.GDXModelBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

public class GDXPlanarRegionLoggingDevelopmentUI
{

   private final String WINDOW_NAME = "Planar Region Logging Development UI";

   private final RDXBaseUI baseUI;

   private ImGuiGDXPlanarRegionLoggingPanel panel;

   private final Timer timer;
   private final Random rand;

   public GDXPlanarRegionLoggingDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", WINDOW_NAME);
      timer = new Timer();
      rand = new Random();
   }

   public void launch()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));

            panel = new ImGuiGDXPlanarRegionLoggingPanel();
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
      GDXPlanarRegionLoggingDevelopmentUI ui = new GDXPlanarRegionLoggingDevelopmentUI();
      ui.launch();
   }
}

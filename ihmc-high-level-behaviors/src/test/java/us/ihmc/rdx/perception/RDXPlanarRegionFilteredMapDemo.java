package us.ihmc.rdx.perception;

import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXPlanarRegionFilteredMapDemo
{
   private PlanarRegionMappingManager mapHandler;
   private PlanarRegionFilteredMapUIPanel planarRegionFilteredMapUIPanel;
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   public RDXPlanarRegionFilteredMapDemo()
   {
      mapHandler = new PlanarRegionMappingManager();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            planarRegionFilteredMapUIPanel = new PlanarRegionFilteredMapUIPanel("Filtered Map", mapHandler);
            baseUI.getImGuiPanelManager().addPanel(planarRegionFilteredMapUIPanel.getImGuiPanel());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionFilteredMapUIPanel::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            if (planarRegionFilteredMapUIPanel.isCaptured())
            {
               LogTools.info("Filtered Map Panel Captured: {}", planarRegionFilteredMapUIPanel.isCaptured());
               mapHandler.setCaptured(true);
               planarRegionFilteredMapUIPanel.setCaptured(false);
            }

            planarRegionFilteredMapUIPanel.renderPlanarRegions();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionFilteredMapDemo();
   }
}

package us.ihmc.rdx.perception;

import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class RDXPlanarRegionFilteredMapDemo
{

   private PlanarRegionMappingManager mapHandler;
   private PlanarRegionFilteredMapUI planarRegionFilteredMapUI;

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   public RDXPlanarRegionFilteredMapDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            mapHandler = new PlanarRegionMappingManager();
            planarRegionFilteredMapUI = new PlanarRegionFilteredMapUI("Filtered Map", mapHandler);
            baseUI.getImGuiPanelManager().addPanel(planarRegionFilteredMapUI.getImGuiPanel());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionFilteredMapUI::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            if(planarRegionFilteredMapUI.isCaptured())
            {
               LogTools.info("Filtered Map Panel Captured: {}", planarRegionFilteredMapUI.isCaptured());
               mapHandler.setCaptured(true);
               planarRegionFilteredMapUI.setCaptured(false);
            }

            planarRegionFilteredMapUI.renderPlanarRegions();

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

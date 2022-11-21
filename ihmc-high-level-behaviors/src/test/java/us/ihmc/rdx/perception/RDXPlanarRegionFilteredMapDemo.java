package us.ihmc.rdx.perception;

import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.tools.thread.Activator;

public class RDXPlanarRegionFilteredMapDemo
{
   private final RDXPlanarRegionsGraphic graphic = new RDXPlanarRegionsGraphic();

   private Activator nativesLoadedActivator;
   private PlanarRegionMappingManager mapHandler;

   private PlanarRegionFilteredMapPanel filteredMapPanel;

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

            filteredMapPanel = new PlanarRegionFilteredMapPanel("Filtered Map", mapHandler);

            baseUI.getImGuiPanelManager().addPanel(filteredMapPanel);

            graphic.generateMeshes(mapHandler.getMapRegions());
            graphic.update();

            baseUI.getPrimaryScene().addRenderableProvider(graphic);
         }

         @Override
         public void render()
         {
            if(filteredMapPanel.isCaptured())
            {
               LogTools.info("Filtered Map Panel Captured: {}", filteredMapPanel.isCaptured());
               mapHandler.setCaptured(true);
               filteredMapPanel.setCaptured(false);
            }

            if (mapHandler.getFilteredMap().isModified() && mapHandler.getMapRegions().getNumberOfPlanarRegions() > 0)
            {
               LogTools.info("Regions Available and Modified: {} {}", mapHandler.getFilteredMap().isModified(), mapHandler.getMapRegions().getNumberOfPlanarRegions());
               graphic.clear();
               graphic.generateMeshes(mapHandler.getMapRegions());
               graphic.update();
               mapHandler.getFilteredMap().setModified(false);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            //graphic.destroy();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionFilteredMapDemo();
   }
}

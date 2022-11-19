package us.ihmc.rdx.perception;

import us.ihmc.perception.mapping.PlanarRegionFilteredMap;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.tools.thread.Activator;

public class RDXPlanarRegionFilteredMapDemo
{
   private final RDXPlanarRegionsGraphic graphic = new RDXPlanarRegionsGraphic();

   private Activator nativesLoadedActivator;
   private PlanarRegionFilteredMap filteredMap;

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   public RDXPlanarRegionFilteredMapDemo()
   {

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            filteredMap = new PlanarRegionFilteredMap();

            graphic.generateMeshes(filteredMap.getMapRegions());
            graphic.update();

            baseUI.getPrimaryScene().addRenderableProvider(graphic);
         }

         @Override
         public void render()
         {
            if (filteredMap.isModified())
            {
               graphic.clear();
               graphic.generateMeshes(filteredMap.getMapRegions());
               graphic.update();
               filteredMap.setModified(false);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            graphic.destroy();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionFilteredMapDemo();
   }
}

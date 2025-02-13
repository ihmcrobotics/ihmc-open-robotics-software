package us.ihmc.rdx;

import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.graphics.RDXPlanarRegionsGraphic;

public class RDXPlanarRegionVisualizerDemo
{
   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   public RDXPlanarRegionVisualizerDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            planarRegionsGraphic = new RDXPlanarRegionsGraphic();

            baseUI.getPrimaryScene().addCoordinateFrame(1.0);

            planarRegionsGraphic.generateMeshes(BehaviorPlanarRegionEnvironments.createUpDownOpenHouseRegions());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionsGraphic);
         }

         @Override
         public void render()
         {
            planarRegionsGraphic.update();
            
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            planarRegionsGraphic.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionVisualizerDemo();
   }
}

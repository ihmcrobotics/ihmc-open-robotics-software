package us.ihmc.gdx;

import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;

public class GDXPlanarRegionVisualizerDemo
{
   private final GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();

   public GDXPlanarRegionVisualizerDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(1.0);

            planarRegionsGraphic.generateMeshes(BehaviorPlanarRegionEnvironments.createUpDownOpenHouseRegions());
            sceneManager.addRenderableProvider(planarRegionsGraphic);
         }

         @Override
         public void render()
         {
            planarRegionsGraphic.update();
            
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }

         @Override
         public void dispose()
         {
            planarRegionsGraphic.destroy();
            sceneManager.dispose();
         }
      }, getClass().getSimpleName(), 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDXPlanarRegionVisualizerDemo();
   }
}

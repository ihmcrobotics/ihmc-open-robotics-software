package us.ihmc.rdx;

import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class RDXPlanarRegionVisualizerDemo
{
   private final RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
   private final RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();

   public RDXPlanarRegionVisualizerDemo()
   {
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
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
      new RDXPlanarRegionVisualizerDemo();
   }
}

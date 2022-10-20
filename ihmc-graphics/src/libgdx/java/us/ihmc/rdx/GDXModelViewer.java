package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.sceneManager.GDX3DBareBonesScene;
import us.ihmc.rdx.tools.GDXApplicationCreator;
import us.ihmc.rdx.tools.GDXModelLoader;

public class GDXModelViewer
{
   public GDXModelViewer(String modelFileName)
   {
      GDX3DBareBonesScene sceneManager = new GDX3DBareBonesScene();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();
            sceneManager.addCoordinateFrame(1.0);

            Model model = GDXModelLoader.load(modelFileName);
            sceneManager.addModelInstance(new ModelInstance(model));
         }

         @Override
         public void render()
         {
            sceneManager.render();
         }
      }, "Model Viewer", 1100, 800);
   }
}

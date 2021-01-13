package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;

public class GDXModelViewer
{
   public GDXModelViewer(String modelFileName)
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();
            sceneManager.addCoordinateFrame(1.0);

            Model model = GDXModelLoader.loadG3DModel(modelFileName);
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

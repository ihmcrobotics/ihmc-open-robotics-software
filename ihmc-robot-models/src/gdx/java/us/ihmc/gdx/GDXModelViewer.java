package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;

public class GDXModelViewer
{
   public GDXModelViewer(String modelFileName)
   {
      GDX3DApplication baseApplication = new GDX3DApplication();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseApplication.create();
            baseApplication.addCoordinateFrame(1.0);

            Model model = GDXModelLoader.loadG3DModel(modelFileName);
            baseApplication.addModelInstance(new ModelInstance(model));
         }

         @Override
         public void render()
         {
            baseApplication.render();
         }
      }, "Model Viewer", 1100, 800);
   }
}

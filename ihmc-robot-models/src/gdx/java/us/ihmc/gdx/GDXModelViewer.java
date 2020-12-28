package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.utils.JsonReader;

public class GDXModelViewer
{
   private final String modelFileName;

   public GDXModelViewer(String modelFileName)
   {
      this.modelFileName = modelFileName;
      GDXApplicationCreator.launchGDXApplication(new PrivateApplication(), "Model Viewer", 1100, 800);
   }

   class PrivateApplication extends GDX3DApplication
   {
      @Override
      public void create()
      {
         super.create();

         addCoordinateFrame(1.0);

         Model model = GDXModelLoader.loadG3DModel(modelFileName);

         addModelInstance(new ModelInstance(model));
      }

      @Override
      public void render()
      {
         super.render();
      }
   }
}

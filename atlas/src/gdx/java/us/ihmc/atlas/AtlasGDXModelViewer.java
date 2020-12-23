package us.ihmc.atlas;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.assets.loaders.resolvers.InternalFileHandleResolver;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import com.badlogic.gdx.utils.JsonReader;
import us.ihmc.gdx.GDX3DApplication;
import us.ihmc.gdx.GDXApplicationCreator;

public class AtlasGDXModelViewer
{
   public AtlasGDXModelViewer()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateApplication(), "Atlas Model Viewer", 1100, 800);
   }

   class PrivateApplication extends GDX3DApplication
   {
      @Override
      public void create()
      {
         super.create();

         addCoordinateFrame(1.0);

         G3dModelLoader g3dModelLoader = new G3dModelLoader(new JsonReader(), new InternalFileHandleResolver());
         FileHandle modelFile = Gdx.files.internal("atlas_torso.g3dj");
         Model model = g3dModelLoader.loadModel(modelFile, new TextureProvider.FileTextureProvider());
         addModelInstance(new ModelInstance(model));
      }

      @Override
      public void render()
      {
         super.render();
      }
   }

   public static void main(String[] args)
   {
      new AtlasGDXModelViewer();
   }
}

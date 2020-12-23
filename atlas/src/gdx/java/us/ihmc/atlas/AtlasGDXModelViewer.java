package us.ihmc.atlas;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.assets.AssetManager;
import com.badlogic.gdx.assets.loaders.resolvers.InternalFileHandleResolver;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import com.badlogic.gdx.utils.JsonReader;
import us.ihmc.gdx.GDX3DApplication;
import us.ihmc.gdx.GDXApplicationCreator;
import us.ihmc.log.LogTools;

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

         G3dModelLoader g3dModelLoader = new G3dModelLoader(new JsonReader(), new InternalFileHandleResolver());
         FileHandle modelFile = Gdx.files.internal("atlas_torso.g3dj");
         boolean modelFileExists = modelFile.exists();
         LogTools.info(modelFileExists);
         FileHandle textureFile = Gdx.files.internal("torso_diffuse_unplugged.jpg");
         boolean textureFileExists = textureFile.exists();
         LogTools.info(textureFileExists);
         LogTools.info(Gdx.files.classpath("torso_diffuse_unplugged.jpg").exists());
         LogTools.info(Gdx.files.internal("hello.txt").exists());
         LogTools.info(Gdx.files.classpath("hello.txt").exists());
         Model model = g3dModelLoader.loadModel(modelFile, new TextureProvider.FileTextureProvider());
         addModelInstance(new ModelInstance(model));

         //         AssetManager assetManager = new AssetManager();
//         assetManager.load("atlas_torso.g3dj", Model.class);
//         assetManager.finishLoadingAsset("atlas_torso.g3dj");
//         assetManager.get("atlas_torso.g3dj", Model.class)
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

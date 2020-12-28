package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.utils.JsonReader;

public class GDXModelLoader
{
   public static Model loadG3DModel(String modelFileName)
   {
      //         G3dModelLoader g3dModelLoader = new G3dModelLoader(new JsonReader(), new InternalFileHandleResolver());
      //         FileHandle modelFile = Gdx.files.internal(modelFileName);
      //         Model model = g3dModelLoader.loadModel(modelFile, new TextureProvider.FileTextureProvider());

      return new G3dModelLoader(new JsonReader()).loadModel(Gdx.files.internal(modelFileName));
   }
}

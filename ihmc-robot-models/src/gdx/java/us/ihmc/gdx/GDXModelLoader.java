package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.utils.*;

public class GDXModelLoader
{
   public static Model loadG3DModel(String modelFileName)
   {
      return new G3dModelLoader(new JsonReader()).loadModel(Gdx.files.internal(modelFileName));
   }
}

package us.ihmc.gdx.tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.utils.*;

import java.util.HashMap;

public class GDXModelLoader
{
   private static final GDXModelLoader modelLoader = new GDXModelLoader();

   private final HashMap<String, Model> loadedModels = new HashMap<>();

   private GDXModelLoader()
   {
   }

   private Model loadOrGetModel(String modelFileName)
   {
      Model model = loadedModels.get(modelFileName);
      if (model == null)
      {
         Model loadedModel = new G3dModelLoader(new JsonReader()).loadModel(Gdx.files.internal(modelFileName));
         loadedModels.put(modelFileName, loadedModel);
         return loadedModel;
      }
      return model;
   }

   private void destroyInternal()
   {
      for (Model loadedModel : loadedModels.values())
      {
         loadedModel.dispose();
      }
      loadedModels.clear();
   }

   public static Model loadG3DModel(String modelFileName)
   {
      return modelLoader.loadOrGetModel(modelFileName);
   }

   public static void destroy()
   {
      modelLoader.destroyInternal();
   }
}

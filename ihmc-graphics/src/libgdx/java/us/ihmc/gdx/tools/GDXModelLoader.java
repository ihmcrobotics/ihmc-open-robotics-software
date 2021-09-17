package us.ihmc.gdx.tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.utils.JsonReader;
import com.badlogic.gdx.utils.SerializationException;
import us.ihmc.log.LogTools;

import java.util.HashMap;

public class GDXModelLoader
{
   private static final GDXModelLoader modelLoader = new GDXModelLoader();

   private final HashMap<String, Model> loadedModels = new HashMap<>();

   private GDXModelLoader()
   {
   }

   public static Model loadG3DModel(String modelFileName)
   {
      return modelLoader.loadOrGetModel(modelFileName);
   }

   public static void destroy()
   {
      modelLoader.destroyInternal();
   }

   private Model loadOrGetModel(String modelFileName)
   {
      Model model = loadedModels.get(modelFileName);
      if (model == null)
      {
         FileHandle fileHandle = Gdx.files.internal(modelFileName);
         try
         {
            Model loadedModel = new G3dModelLoader(new JsonReader()).loadModel(fileHandle);
            for (Material material : loadedModel.materials)
            {
               if (!material.has(TextureAttribute.Diffuse))
               {
                  LogTools.warn("Material \"" + material.id + "\" in model \"" + modelFileName + "\" does not contain TextureAttribute Diffuse. Creating...");

                  Pixmap map = new Pixmap(100, 100, Pixmap.Format.RGBA8888);
                  map.setColor(((ColorAttribute) material.get(ColorAttribute.Diffuse)).color);
                  map.drawRectangle(0, 0, 100, 100);

                  material.set(TextureAttribute.createDiffuse(new Texture(map)));

                  map.dispose();
               }
            }

            loadedModels.put(modelFileName, loadedModel);
            return loadedModel;
         }
         catch (SerializationException | NullPointerException e)
         {
            return null;
         }
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

   /**
    * TODO: Implement Collada, STL, etc model loaders
    */
   public static String modifyFileName(String modelFileName)
   {
      String lowerCase = modelFileName.toLowerCase();
      if (!lowerCase.endsWith(".obj") && !lowerCase.endsWith(".stl"))
      {
         LogTools.warn("Model file \"{}\" is not an OBJ or STL. Skipping...", modelFileName);
         return null;
      }

      String[] splitSlash = modelFileName.split("/");
      String objFileName = splitSlash[splitSlash.length - 1];
      String modifiedFileName = objFileName.replace(".obj", "").replace(".STL", "") + ".g3dj";
      return modifiedFileName;
   }
}

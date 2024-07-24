package us.ihmc.rdx.tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMaterial;
import com.badlogic.gdx.utils.JsonReader;
import com.badlogic.gdx.utils.SerializationException;
import com.badlogic.gdx.utils.UBJsonReader;
import net.mgsx.gltf.loaders.gltf.GLTFLoader;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import net.mgsx.gltf.scene3d.scene.SceneAsset;
import us.ihmc.rdx.tools.assimp.RDXAssimpModelLoader;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.util.HashMap;
import java.util.TreeSet;

public class RDXModelLoader
{
   private static final RDXModelLoader modelLoader = new RDXModelLoader();

   private final HashMap<String, Object> modelLoadingSynchronizers = new HashMap<>();
   private final HashMap<String, Model> loadedModels = new HashMap<>();
   private final TreeSet<String> printedWarnings = new TreeSet<>();

   private RDXModelLoader()
   {
   }

   /**
    * Synchronized (thread-safe) and won't load a model more than once.
    */
   public static Model load(String modelFileName)
   {
      return modelLoader.loadOrGetModel(modelFileName);
   }

   /**
    * No synchronization and will load data from file every time.
    */
   public static ModelData loadModelData(String modelFileName)
   {
      return modelLoader.loadModelDataInternal(modelFileName);
   }

   public static void destroy()
   {
      modelLoader.destroyInternal();
   }

   private Model loadOrGetModel(String modelFileName)
   {
      modelFileName = ResourceTools.sanitizeResourcePath(modelFileName);

      Object preventLoadingMoreThanOnceSynchronizer = modelLoadingSynchronizers.computeIfAbsent(modelFileName, key -> new Object());

      Model model;
      synchronized (preventLoadingMoreThanOnceSynchronizer)
      {
         model = loadedModels.get(modelFileName);
         if (model == null)
         {
            ModelData modelData = loadModelData(modelFileName);
            if (modelData != null)
            {
               model = new Model(modelData);
               ensureModelHasDiffuseTextureAttribute(modelFileName, model);
               loadedModels.put(modelFileName, model);
            }
         }
      }

      return model;
   }

   private ModelData loadModelDataInternal(String modelFileName)
   {
      LogTools.debug("Loading {}", modelFileName);

      String requestedModelFileName = modelFileName;
      boolean shouldPrintWarnings = !printedWarnings.contains(requestedModelFileName);
     
      ModelData modelData = null;
      try
      {
         modelFileName = useABetterFormatIfAvailable(modelFileName);

         if (modelFileName.endsWith(".gltf"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            SceneAsset sceneAsset = new GLTFLoader().load(fileHandle, true);
//            model = sceneAsset.scene.model;
//            sceneAsset.get
         }
         else if (modelFileName.endsWith(".g3dj"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            modelData = new G3dModelLoader(new JsonReader()).loadModelData(fileHandle);
         }
         else if (modelFileName.endsWith(".g3db"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            modelData = new G3dModelLoader(new UBJsonReader()).loadModelData(fileHandle);
         }
         else
         {
            if (shouldPrintWarnings)
               LogTools.warn("Using Assimp to load {}. It is recommended to convert to G3DJ for more reliable and faster loading.", modelFileName);
            modelData = new RDXAssimpModelLoader(modelFileName).loadModelData();
         }

         long numberOfVertices = LibGDXTools.countVertices(modelData);
         LogTools.debug("Loaded {} ({} vertices)", modelFileName, numberOfVertices);

         if (shouldPrintWarnings && numberOfVertices > 15000)
         {
            LogTools.warn("{} has {} vertices, which is a lot! This will begin to affect frame rate.", modelFileName, numberOfVertices);
         }

      }
      catch (SerializationException | NullPointerException e)
      {
         LogTools.error("Failed to load {}", modelFileName);
         e.printStackTrace();
      }

      printedWarnings.add(requestedModelFileName);

      return modelData;
   }

   public static void ensureModelHasDiffuseTextureAttribute(String modelFileName, Model model)
   {
      for (Material material : model.materials)
      {
         if (!material.has(TextureAttribute.Diffuse))
         {
            LogTools.debug(
                  "Material \"" + material.id + "\" in model \"" + modelFileName + "\" does not contain TextureAttribute Diffuse. Creating...");

            Pixmap map = new Pixmap(100, 100, Pixmap.Format.RGBA8888);
            map.setColor(((ColorAttribute) material.get(ColorAttribute.Diffuse)).color);
            map.drawRectangle(0, 0, 100, 100);

            material.set(PBRTextureAttribute.createBaseColorTexture(new Texture(map)));

            map.dispose();
         }
      }
   }

   private String useABetterFormatIfAvailable(String modelFileName)
   {
      boolean gltfExists = false;
      // TODO: Possibly figure out msgx's gltf support
      //  api("com.github.mgsx-dev.rdx-gltf:gltf:2.0.0-rc.1")
      // if (!modelFileName.endsWith(".gltf"))
      // {
      //    String modelFileNameWithoutExtension = modelFileName.substring(0, modelFileName.lastIndexOf("."));
      //    FileHandle potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".gltf");
      //    if (potentialFileHandle.exists())
      //    {
      //       LogTools.debug("Found glTF 2.0 file as an alternative for {}", modelFileName);
      //       modelFileName = modelFileNameWithoutExtension + ".gltf";
      //       gltfExists = true;
      //    }
      // }

      boolean g3dbExists = false;
      if (!gltfExists && !modelFileName.endsWith(".g3db"))
      {
         String modelFileNameWithoutExtension = modelFileName.substring(0, modelFileName.lastIndexOf("."));
         FileHandle potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".g3db");
         if (potentialFileHandle.exists())
         {
            LogTools.debug("Found G3DB file as an alternative for {}", modelFileName);
            modelFileName = modelFileNameWithoutExtension + ".g3db";
            g3dbExists = true;
         }
      }

      if (!gltfExists && !g3dbExists && !modelFileName.endsWith(".g3dj"))
      {
         String modelFileNameWithoutExtension = modelFileName.substring(0, modelFileName.lastIndexOf("."));
         FileHandle potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".g3dj");
         if (potentialFileHandle.exists())
         {
            LogTools.debug("Found G3DJ file as an alternative for {}", modelFileName);
            modelFileName = modelFileNameWithoutExtension + ".g3dj";
         }
      }

      return modelFileName;
   }

   private void destroyInternal()
   {
      for (Model loadedModel : loadedModels.values())
      {
         loadedModel.dispose();
      }
      loadedModels.clear();
   }
}

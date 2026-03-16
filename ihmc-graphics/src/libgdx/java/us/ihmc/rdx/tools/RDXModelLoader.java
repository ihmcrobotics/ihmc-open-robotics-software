package us.ihmc.rdx.tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.utils.JsonReader;
import com.badlogic.gdx.utils.SerializationException;
import com.badlogic.gdx.utils.UBJsonReader;
import net.mgsx.gltf.loaders.glb.GLBLoader;
import net.mgsx.gltf.loaders.gltf.GLTFLoader;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import net.mgsx.gltf.scene3d.scene.SceneAsset;
import us.ihmc.rdx.tools.assimp.RDXAssimpModelLoader;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Paths;
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
   public static Model loadForce(String modelFileName)
   {
      return modelLoader.loadModelInternal(modelFileName);
   }

   public static void destroy()
   {
      modelLoader.destroyInternal();
   }

   private Model loadOrGetModel(String modelFileName)
   {
      String requestedModelFileName = modelFileName;

      if (!Paths.get(modelFileName).isAbsolute()) // Ignore in the case of absolute file paths. TODO: What is the point of sanitizeResourcePath?
         modelFileName = ResourceTools.sanitizeResourcePath(modelFileName);

      Object preventLoadingMoreThanOnceSynchronizer = modelLoadingSynchronizers.computeIfAbsent(modelFileName, key -> new Object());

      Model model;
      synchronized (preventLoadingMoreThanOnceSynchronizer)
      {
         model = loadedModels.get(modelFileName);
         if (model == null)
         {
            model = loadModelInternal(requestedModelFileName);
            ensureModelIsPBRShaderCompatible(model);
            loadedModels.put(modelFileName, model);
         }
         else
         {
            boolean shouldPrintWarnings = !printedWarnings.contains(requestedModelFileName);

            long numberOfVertices = LibGDXTools.countVertices(model);
            LogTools.debug("Loaded {} ({} vertices)", modelFileName, numberOfVertices);

            if (shouldPrintWarnings && numberOfVertices > 15000)
            {
               LogTools.warn("{} has {} vertices, which is a lot! This will begin to affect frame rate.", modelFileName, numberOfVertices);
            }

            printedWarnings.add(requestedModelFileName);

            return model;
         }
      }

      return model;
   }

   private Model loadModelInternal(String modelFileName)
   {
      LogTools.debug("Loading {}", modelFileName);

      String requestedModelFileName = modelFileName;
      boolean shouldPrintWarnings = !printedWarnings.contains(requestedModelFileName);
     
      Model model = null;
      try
      {
         modelFileName = useABetterFormatIfAvailable(modelFileName);

         if (modelFileName.endsWith(".glb"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            SceneAsset sceneAsset = new GLBLoader().load(fileHandle, true);
            model = sceneAsset.scene.model;
         }
         else if (modelFileName.endsWith(".gltf"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            SceneAsset sceneAsset = new GLTFLoader().load(fileHandle, true);
            model = sceneAsset.scene.model;
         }
         else if (modelFileName.endsWith(".g3dj"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            model = new G3dModelLoader(new JsonReader()).loadModel(fileHandle);
         }
         else if (modelFileName.endsWith(".g3db"))
         {
            FileHandle fileHandle = Gdx.files.internal(modelFileName);
            model = new G3dModelLoader(new UBJsonReader()).loadModel(fileHandle);
         }
         else
         {
            if (shouldPrintWarnings)
               LogTools.warn("Using Assimp to load {}. It is recommended to convert to glTF binary (*.glb) for "
                             + "better rendering and more reliable and faster loading.", modelFileName);
            model = new RDXAssimpModelLoader(modelFileName).load();
         }

         long numberOfVertices = LibGDXTools.countVertices(model);
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

      return model;
   }

   public static void ensureModelIsPBRShaderCompatible(Model model)
   {
      for (Material material : model.materials)
      {
         ColorAttribute diffuseColor = material.get(ColorAttribute.class, ColorAttribute.Diffuse);
         TextureAttribute diffuseTexture = material.get(TextureAttribute.class, TextureAttribute.Diffuse);

         boolean diffuseColorNeedsReplacement = diffuseColor != null && !(diffuseColor instanceof PBRColorAttribute);
         boolean diffuseTextureNeedsReplacement = diffuseTexture != null && !(diffuseTexture instanceof PBRTextureAttribute);

         if (diffuseColorNeedsReplacement || diffuseTextureNeedsReplacement)
         {
            material.clear();

            if (diffuseTextureNeedsReplacement)
            {
               material.set(PBRTextureAttribute.createBaseColorTexture(diffuseTexture.textureDescription.texture));
            }
            if (diffuseColorNeedsReplacement)
            {
               material.set(PBRColorAttribute.createBaseColorFactor(diffuseColor.color));
            }
         }
      }
   }

   private String useABetterFormatIfAvailable(String modelFileName)
   {
      String modelFileNameWithoutExtension = modelFileName.substring(0, modelFileName.lastIndexOf("."));

      // Most preferable first
      if ((modelFileName.endsWith(".glb") || modelFileName.endsWith(".gltf")) && Gdx.files.internal(modelFileName).exists())
         return modelFileName;
      else
      {
         FileHandle potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".glb");
         if (potentialFileHandle.exists())
         {
            LogTools.debug("Found GLB file as an alternative for {}", modelFileName);
            modelFileName = modelFileNameWithoutExtension + ".glb";
            return modelFileName;
         }

         potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".gltf");
         if (potentialFileHandle.exists())
         {
            LogTools.debug("Found GLTF file as an alternative for {}", modelFileName);
            modelFileName = modelFileNameWithoutExtension + ".gltf";
            return modelFileName;
         }
      }

      if ((modelFileName.endsWith(".g3db") || modelFileName.endsWith(".g3dj")) && Gdx.files.internal(modelFileName).exists())
         return modelFileName;
      else
      {
         FileHandle potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".g3db");
         if (potentialFileHandle.exists())
         {
            LogTools.debug("Found G3DB file as an alternative for {}", modelFileName);
            modelFileName = modelFileNameWithoutExtension + ".g3db";
            return modelFileName;
         }

         potentialFileHandle = Gdx.files.internal(modelFileNameWithoutExtension + ".g3dj");
         if (potentialFileHandle.exists())
         {
            LogTools.debug("Found G3DJ file as an alternative for {}", modelFileName);
            modelFileName = modelFileNameWithoutExtension + ".g3dj";
            return modelFileName;
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

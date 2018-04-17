package us.ihmc.jMonkeyEngineToolkit.jme;

import java.io.File;

import com.jme3.asset.AssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.material.Material;
import com.jme3.material.MaterialList;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.ogre.matext.MaterialExtension;
import com.jme3.scene.plugins.ogre.matext.MaterialExtensionSet;
import com.jme3.scene.plugins.ogre.matext.OgreMaterialKey;
import com.jme3.texture.Texture;

public class JMEAssetLocator
{
   private static boolean ogreLoaderInitialized = false;
   private static AssetManager assetManager;

   public JMEAssetLocator(AssetManager assetManager)
   {
      JMEAssetLocator.assetManager = assetManager;
      setupAssetManagerPaths(assetManager);

   }

   public static Spatial loadModel(String fileName)
   {
      return assetManager.loadModel(fileName);
   }

   public static Texture loadTexture(String name)
   {
      return assetManager.loadTexture(name);
   }

   public static AssetManager getAssetManager()
   {
      return assetManager;
   }

   public static Material loadMaterial(String name)
   {
      return assetManager.loadMaterial(name);
   }

   public static Object loadAsset(String name)
   {
      return assetManager.loadAsset(name);
   }

   public static void setupAssetManagerPaths(AssetManager assetManager)
   {
      assetManager.registerLocator("/", ClasspathLocator.class);
      File[] roots = File.listRoots();
      for (File root : roots)
      {
         if (root.canRead())
            assetManager.registerLocator(root.getAbsolutePath(), FileLocator.class);
      }
   }

   public static MaterialList loadOgreAsset(String name, AssetManager assetManager)
   {
      if (!ogreLoaderInitialized)
      {
         assetManager.unregisterLoader(com.jme3.scene.plugins.ogre.MaterialLoader.class);
         assetManager.registerLoader(com.jme3.scene.plugins.ogre.MaterialLoader.class, "material");
         ogreLoaderInitialized = true;
      }

      // Normalize the path stuff on Windows
      name = name.replace("\\", "/");

      OgreMaterialKey ogreMaterialKey = new OgreMaterialKey(name);
      MaterialExtensionSet matExts = new MaterialExtensionSet();
      matExts.addMaterialExtension(new MaterialExtension("", ""));

      ogreMaterialKey.setMaterialExtensionSet(matExts);

      return assetManager.loadAsset(ogreMaterialKey);
   }

}

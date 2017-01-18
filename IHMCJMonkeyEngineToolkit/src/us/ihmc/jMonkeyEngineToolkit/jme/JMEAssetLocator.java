package us.ihmc.jMonkeyEngineToolkit.jme;

import java.io.File;

import org.jmonkeyengine.scene.plugins.ogre.matext.MaterialExtension;
import org.jmonkeyengine.scene.plugins.ogre.matext.MaterialExtensionSet;
import org.jmonkeyengine.scene.plugins.ogre.matext.OgreMaterialKey;

import com.jme3.asset.AssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.material.Material;
import com.jme3.material.MaterialList;
import com.jme3.scene.Spatial;
import com.jme3.texture.Texture;

public class JMEAssetLocator
{
   private boolean ogreLoaderInitialized = false;
   private AssetManager assetManager;

   public JMEAssetLocator(AssetManager assetManager)
   {
      this.assetManager = assetManager;

      assetManager.registerLocator("/", ClasspathLocator.class);
      File[] roots = File.listRoots();
      for(File root : roots)
      {
          if(root.canRead())
              assetManager.registerLocator(root.getAbsolutePath(), FileLocator.class);
      }
   }

   public Spatial loadModel(String fileName)
   {
      return assetManager.loadModel(fileName);
   }

   public Texture loadTexture(String name)
   {
      return assetManager.loadTexture(name);
   }

   public AssetManager getAssetManager()
   {
      return assetManager;
   }

   public Material loadMaterial(String name)
   {
      return assetManager.loadMaterial(name);
   }
   
   public Object loadAsset(String name)
   {
      return assetManager.loadAsset(name);
   }
   
   public MaterialList loadOgreAsset(String name)
   {
      if(!ogreLoaderInitialized)
      {
         assetManager.unregisterLoader(com.jme3.scene.plugins.ogre.MaterialLoader.class);
         assetManager.registerLoader(org.jmonkeyengine.scene.plugins.ogre.MaterialLoader.class, "material");
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

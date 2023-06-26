package us.ihmc.avatar.ConcaveMeshes;

import com.jme3.asset.AssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.audio.AudioData;
import com.jme3.audio.AudioKey;
import com.jme3.font.BitmapFont;
import com.jme3.material.Material;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.Transform;
import com.jme3.post.FilterPostProcessor;
import com.jme3.renderer.Caps;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.OBJLoader;
import com.jme3.scene.plugins.gltf.GltfLoader;
import com.jme3.shader.ShaderGenerator;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.TGALoader;

import eu.mihosoft.vrl.v3d.Main;
import vhacd4.Vhacd4Parameters;

import java.io.IOException;
import java.io.InputStream;
import java.util.EnumSet;
import java.util.List;

import com.jme3.asset.DesktopAssetManager;
public class MeshLoader
{
   public MeshLoader()
   {
   
   }
   
   public void LoadFile()
   {
      
      AssetManager assetManager = new DesktopAssetManager();
      assetManager.registerLoader(GltfLoader.class, "gltf");
      assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
    
      assetManager.registerLocator("C:/Users/Khizar/Desktop/Terrain", FileLocator.class);
      assetManager.registerLocator(null, ClasspathLocator.class);
      
      
      Spatial cgmRoot = assetManager.loadModel("doorway.gltf");
      Spatial parent = ((Node) cgmRoot).getChild(0);
      parent.setLocalTransform(Transform.IDENTITY);
      Spatial geom = ((Node) parent).getChild(0);
      
      Vhacd4Parameters parameters = new Vhacd4Parameters();
      parameters.setMaxVerticesPerHull(99);
      parameters.setVoxelResolution(900_000);
      
      
//      CompoundCollisionShape shape = ShapeUtils.createVhacdShape(cgmRoot, parameters, "MakeDuck");
      
      
   }
   
   
   
   public static void main(String[] args)
   {
      MeshLoader mesh =new MeshLoader();
      mesh.LoadFile();
   }
}

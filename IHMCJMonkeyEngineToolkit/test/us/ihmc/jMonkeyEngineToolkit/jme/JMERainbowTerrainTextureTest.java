package us.ihmc.jMonkeyEngineToolkit.jme;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.material.Material;
import com.jme3.scene.Geometry;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.HillHeightMap;

import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;

public class JMERainbowTerrainTextureTest extends SimpleApplication
{
   int counter = 0;
   

   Geometry footIconGeometryLeft;
   public void simpleInitApp()
   {
      String path = JMERenderer.class.getResource("Resources").getPath();
      assetManager.registerLocator(path, FileLocator.class);
      
      
      flyCam.setDragToRotate(true);
      flyCam.setMoveSpeed(100.0f);
      HeightMap heightmap;
      try
      {
         heightmap = new HillHeightMap(1025, 1000, 50, 100, (byte) 3);
         TerrainQuad terrain = new TerrainQuad("terrain", 65, 1025, heightmap.getHeightMap());
         
         Material matTerrain;

         matTerrain = new Material(assetManager, "Terrain/RainbowHeightTerrain.j3md");
         matTerrain.setFloat("maxHeight", 200.0f);
         matTerrain.setFloat("minHeight", -100.0f);
         terrain.setMaterial(matTerrain);
         
         rootNode.attachChild(terrain);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      
   }
   
   public static void main(String[] args)
   {
      JMERainbowTerrainTextureTest jmeRainbowTerrainTextureTest = new JMERainbowTerrainTextureTest();
      AppSettings appSettings = new AppSettings(true);
      appSettings.setResolution(800, 600);
      jmeRainbowTerrainTextureTest.setSettings(appSettings);


      jmeRainbowTerrainTextureTest.setShowSettings(false);
      jmeRainbowTerrainTextureTest.setPauseOnLostFocus(false);
      jmeRainbowTerrainTextureTest.start();
      
   }
}

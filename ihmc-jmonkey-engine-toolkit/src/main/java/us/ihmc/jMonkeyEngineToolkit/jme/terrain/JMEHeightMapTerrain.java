package us.ihmc.jMonkeyEngineToolkit.jme.terrain;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.WrapMode;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;

public class JMEHeightMapTerrain
{
   private final int GRID_SIZE_POWER_OF_TWO = 9;
   
   private final int gridSize = ((int) Math.pow(2, GRID_SIZE_POWER_OF_TWO)) + 1;
   private final int patchSize = ((int) Math.pow(2, GRID_SIZE_POWER_OF_TWO/2)) + 1;
 
   
   private final HeightMap heightMap;
   private final Node terrainNode = new Node("terrainNode");
   private final float xMin, xMax, yMin, yMax;
   private final float width, length;
   private final Vector3f scale = new Vector3f();
   
   
   
   private float getHeightAtIndex(int i)
   {
      float x = (i / gridSize);
      float y = (i % gridSize);
      
      float xCoordinate = scale.x * x + xMin;
      float yCoordinate = scale.y * y + yMin;
      
      return (float) heightMap.heightAt(xCoordinate, yCoordinate, 0.0);
   }
   int xLast = 0, yLast = 0;
   
   public JMEHeightMapTerrain(HeightMap heightMap, AssetManager assetManager)
   {
      this(heightMap, assetManager, null);
   }
   
   public JMEHeightMapTerrain(HeightMap heightMap, AssetManager assetManager, Material material)
   {
      this.heightMap = heightMap;
      BoundingBox3D boundingBox = heightMap.getBoundingBox();
      
      xMin = (float) boundingBox.getMinX();
      xMax = (float) boundingBox.getMaxX();
                            
      yMin = (float) boundingBox.getMinY();
      yMax = (float) boundingBox.getMaxY();
      
      width = xMax - xMin;
      length = yMax - yMin;
      
      if(width < 1e-5 || length < 1e-5)
      {
         return;
      }
      
      scale.x = width / gridSize;
      scale.y = length / gridSize;
      scale.z = 1.0f;
      Vector3f localScale = new Vector3f(scale);
      JMEGeometryUtils.transformFromZupToJMECoordinates(localScale);
      
      float[] floatHeightMap = new float[gridSize * gridSize];
      
      for(int i = 0; i < floatHeightMap.length; i++)
      {
         floatHeightMap[i] = getHeightAtIndex(i);
      }

      
      TerrainQuad terrain = new TerrainQuad("JMEHeightMapTerrain", patchSize, gridSize, floatHeightMap);

      /** 4. We give the terrain its material, position & scale it, and attach it. */
     
      if (material == null)
      {
         material = loadTexture(assetManager, localScale);
      }
      terrain.setMaterial(material);
      
      terrain.setLocalScale(localScale);
      
      //TODO: double check translation
      float xTranslation =  width / 2.0f + xMin;
      float yTranslation =  length / 2.0f + yMin;
      
      terrainNode.setLocalTranslation(xTranslation, yTranslation, 0);
      
      Node rotation = new Node();
      rotation.rotate(JMEGeometryUtils.getRotationFromZupToJMECoordinates());
      terrainNode.attachChild(rotation);
      rotation.attachChild(terrain);
   }
   
   private Material loadTexture(AssetManager assetManager, Vector3f localScale)
   {
      Material matTerrain;

      matTerrain = new Material(assetManager, "Terrain/ScalableTextureTerrain.j3md");
      matTerrain.setBoolean("useTriPlanarMapping", true);

      Texture texture = assetManager.loadTexture("Textures/gridGroundProfile.png");
      texture.setWrap(WrapMode.Repeat);
      
      matTerrain.setTexture("Texture", texture);
      
      Vector2f scale = new Vector2f(localScale.x, localScale.z);
      matTerrain.setFloat("gridSize", gridSize);
      matTerrain.setVector2("Scale", scale);
           
      return matTerrain;
   }
   
   
   public Node getTerrain()
   {
      return terrainNode;
   }
}

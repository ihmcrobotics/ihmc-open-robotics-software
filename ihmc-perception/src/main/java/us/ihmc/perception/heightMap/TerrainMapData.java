package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class TerrainMapData
{
   private RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();

   private int cellsPerMeter = 50;
   private int gridSize = 201;
   private int heightScaleFactor = 10000;

   private final Mat heightMap;
   private final Mat contactMap;
   private final Mat terrainCostMap;

   public TerrainMapData(Mat heightMap, Mat contactMap, Mat terrainCostMap)
   {
      this.heightMap = heightMap;
      this.contactMap = contactMap;
      this.terrainCostMap = terrainCostMap;
   }

   public TerrainMapData(TerrainMapData data)
   {
      this.heightMap = data.heightMap.clone();
      this.contactMap = data.contactMap.clone();
      this.terrainCostMap = data.terrainCostMap.clone();

      this.cellsPerMeter = data.cellsPerMeter;
      this.gridSize = data.gridSize;
      this.heightScaleFactor = data.heightScaleFactor;
   }

   public float getHeightAt(int x, int y)
   {
      return (float) ((heightMap.ptr(x, y).getShort() & 0xFFFF));
   }

   public Mat getHeightMap()
   {
      return heightMap;
   }

   public Mat getContactMap()
   {
      return contactMap;
   }

   public Mat getTerrainCostMap()
   {
      return terrainCostMap;
   }

   public int getCellsPerMeter()
   {
      return cellsPerMeter;
   }

   public int getGridSize()
   {
      return gridSize;
   }
}

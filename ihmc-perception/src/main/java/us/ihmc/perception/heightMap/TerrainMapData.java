package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;

public class TerrainMapData
{
   /**
    * Sensor origin that defines the center of the height map
    */
   private Point2D sensorOrigin;
   private RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();

   private int cellsPerMeter = 50;
   private int gridSize = 201;
   private int heightScaleFactor = 10000;
   private float heightOffset = 3.2768f;

   private Mat heightMap;
   private Mat contactMap;
   private Mat terrainCostMap;

   public TerrainMapData(Mat heightMap, Mat contactMap, Mat terrainCostMap)
   {
      this.heightMap = heightMap;
      this.contactMap = contactMap;
      this.terrainCostMap = terrainCostMap;

      this.gridSize = heightMap.rows();
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

   public TerrainMapData(int height, int width)
   {
      heightMap = new Mat(height, width, opencv_core.CV_16UC1);
      terrainCostMap = new Mat(height, width, opencv_core.CV_8UC1);
      contactMap = new Mat(height, width, opencv_core.CV_8UC1);
   }

   public void set(TerrainMapData terrainMapData)
   {
      this.heightMap = terrainMapData.getHeightMap().clone();
      this.contactMap = terrainMapData.getContactMap().clone();
      this.terrainCostMap = terrainMapData.getTerrainCostMap().clone();
   }

   public float getHeightAt(int rIndex, int cIndex)
   {
      int height = ((int) heightMap.ptr(rIndex, cIndex).getShort() & 0xFFFF);
      float cellHeight = (float) ((float) height / RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor())
                          - (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset();
      return cellHeight;
   }

   public float getHeightInWorld()
   {

   }

   public float getContactScoreAt(int rIndex, int cIndex)
   {
      return (float) ((heightMap.ptr(rIndex, cIndex).get() & 0xFF));
   }

   public void setSensorOrigin(double originX, double originY)
   {
      this.sensorOrigin.set(originX, originY);
   }

   public Point2D getSensorOrigin() { return sensorOrigin; }

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

   public void setHeightMap(Mat heightMap)
   {
      this.heightMap = heightMap;
   }

   public void setContactMap(Mat contactMap)
   {
      this.contactMap = contactMap;
   }

   public void setTerrainCostMap(Mat terrainCostMap)
   {
      this.terrainCostMap = terrainCostMap;
   }
}

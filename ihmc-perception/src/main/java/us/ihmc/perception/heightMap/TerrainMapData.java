package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class TerrainMapData
{
   /**
    * Sensor origin that defines the center of the height map
    */
   private final Point2D sensorOrigin = new Point2D();
   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();

   private int cellsPerMeter = 50;
   private int localGridSize = 201;
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

      this.localGridSize = heightMap.rows();
   }

   public TerrainMapData(TerrainMapData data)
   {
      this.heightMap = data.heightMap.clone();
      this.contactMap = data.contactMap.clone();
      this.terrainCostMap = data.terrainCostMap.clone();

      this.cellsPerMeter = data.cellsPerMeter;
      this.localGridSize = data.localGridSize;
      this.heightScaleFactor = data.heightScaleFactor;
      this.heightOffset = data.heightOffset;
      this.sensorOrigin.set(data.sensorOrigin);
      this.zUpToWorldTransform.set(data.zUpToWorldTransform);
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

   public int getLocalIndex(float coordinate, float center)
   {
      int localIndex = (int) ((coordinate - center) * cellsPerMeter + localGridSize / 2);
      return localIndex;
   }

   public float getHeightLocal(int rIndex, int cIndex)
   {
      if (rIndex < 0 || rIndex >= localGridSize || cIndex < 0 || cIndex >= localGridSize)
         return 0.0f;

      int height = ((int) heightMap.ptr(rIndex, cIndex).getShort() & 0xFFFF);
      float cellHeight = (float) ((float) height / RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor())
                          - (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset();
      return cellHeight;
   }

   public float getHeightInWorld(float x, float y)
   {
      int rIndex = getLocalIndex(x, sensorOrigin.getX32());
      int cIndex = getLocalIndex(y, sensorOrigin.getY32());
      return getHeightLocal(rIndex, cIndex);
   }

   public float getContactScoreLocal(int rIndex, int cIndex)
   {
      if (rIndex < 0 || rIndex >= localGridSize || cIndex < 0 || cIndex >= localGridSize)
         return 0.0f;

      return (float) ((heightMap.ptr(rIndex, cIndex).get() & 0xFF));
   }

   public float getContactScoreInWorld(float x, float y)
   {
      int rIndex = getLocalIndex(x, sensorOrigin.getX32());
      int cIndex = getLocalIndex(y, sensorOrigin.getY32());
      return getContactScoreLocal(rIndex, cIndex);
   }

   public void setHeightLocal(float height, int rIndex, int cIndex)
   {
      float offsetHeight = height - (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset();
      int finalHeight = (int) (offsetHeight * RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor());
      heightMap.ptr(rIndex, cIndex).putShort((short) finalHeight);
   }

   public void fillHeightMap(HeightMapData heightMapData)
   {
      this.sensorOrigin.set(heightMapData.getGridCenter());
      int centerIndex = heightMapData.getCenterIndex();
      for (int i = 0; i < heightMapData.getNumberOfOccupiedCells(); i++)
      {
         int key = heightMapData.getKey(i);
         int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);
         double height = heightMapData.getHeightAt(key);
         setHeightLocal((float) height, xIndex, yIndex);
      }
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

   public int getLocalGridSize()
   {
      return localGridSize;
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

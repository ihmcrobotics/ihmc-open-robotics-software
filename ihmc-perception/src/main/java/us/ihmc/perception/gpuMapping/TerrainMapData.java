package us.ihmc.perception.gpuMapping;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

import java.util.ArrayList;
import java.util.Arrays;

public class TerrainMapData
{
   private final double cellSize;
   private final double mapSize;
   private double gridCenterX;
   private double gridCenterY;
   private final int centerIndex;
   private final int cellsPerAxis;

   private final float[] heightMap;

   private final float[] traversabilityScoreMap;
   private final byte[] traversabilityClassMap;

   private final byte[] snapNormalXMap;
   private final byte[] snapNormalYMap;
   private final byte[] snapNormalZMap;

   public TerrainMapData(double cellSize, double mapSize, double gridCenterX, double gridCenterY)
   {
      this.cellSize = cellSize;
      this.mapSize = mapSize;
      this.gridCenterX = gridCenterX;
      this.gridCenterY = gridCenterY;

      this.centerIndex = HeightMapTools.computeCenterIndex(mapSize, cellSize);
      this.cellsPerAxis = 2 * centerIndex + 1;

      heightMap = new float[cellsPerAxis * cellsPerAxis];

      traversabilityScoreMap = new float[cellsPerAxis * cellsPerAxis];
      traversabilityClassMap = new byte[cellsPerAxis * cellsPerAxis];

      snapNormalXMap = new byte[cellsPerAxis * cellsPerAxis];
      snapNormalYMap = new byte[cellsPerAxis * cellsPerAxis];
      snapNormalZMap = new byte[cellsPerAxis * cellsPerAxis];

      // Initialize the snap normal as ZUp
      byte zUpNormalXY = packFloatAsByte(0.0f, -1.0f, 1.0f);
      byte zUpNormalZ = packFloatAsByte(1.0f, 0.0f, 1.0f);
      Arrays.fill(snapNormalXMap, zUpNormalXY);
      Arrays.fill(snapNormalYMap, zUpNormalXY);
      Arrays.fill(snapNormalZMap, zUpNormalZ);
   }

   public TerrainMapData(TerrainMapData other)
   {
      this.cellSize = other.cellSize;
      this.mapSize = other.mapSize;
      this.gridCenterX = other.gridCenterX;
      this.gridCenterY = other.gridCenterY;

      this.centerIndex = HeightMapTools.computeCenterIndex(mapSize, cellSize);
      this.cellsPerAxis = 2 * centerIndex + 1;

      int size = cellsPerAxis * cellsPerAxis;

      this.heightMap = Arrays.copyOf(other.heightMap, size);

      this.traversabilityScoreMap = Arrays.copyOf(other.traversabilityScoreMap, size);
      this.traversabilityClassMap = Arrays.copyOf(other.traversabilityClassMap, size);

      this.snapNormalXMap = Arrays.copyOf(other.snapNormalXMap, size);
      this.snapNormalYMap = Arrays.copyOf(other.snapNormalYMap, size);
      this.snapNormalZMap = Arrays.copyOf(other.snapNormalZMap, size);
   }

   public void checkHeightMapSize(double cellSize, double mapSize)
   {
      if (this.cellSize != cellSize)
         throw new RuntimeException("The cell size of the maps are different. Expected " + this.cellSize + ", received  " + cellSize);
      if (this.mapSize != mapSize)
         throw new RuntimeException("The map size of the maps are different. Expected " + this.mapSize + ", received  " + mapSize);
   }

   public void setHeight(double x, double y, double z)
   {
      int key = HeightMapTools.coordinateToKey(x, y, gridCenterX, gridCenterY, cellSize, centerIndex);
      heightMap[key] = (float) z;
   }

   public void setSnapNormal(double x, double y, double normalX, double normalY, double normalZ)
   {
      int key = HeightMapTools.coordinateToKey(x, y, gridCenterX, gridCenterY, cellSize, centerIndex);
      snapNormalXMap[key] = packFloatAsByte((float) normalX, -1.0f, 1.0f);
      snapNormalYMap[key] = packFloatAsByte((float) normalY, -1.0f, 1.0f);
      snapNormalZMap[key] = packFloatAsByte((float) normalZ, 0.0f, 1.0f);
   }

   public double getHeight(double x, double y)
   {
      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenterX, cellSize, centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenterY, cellSize, centerIndex);
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, xIndex, yIndex))
         return Double.NaN;

      int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return heightMap[key];
   }

   public double getHeight(int xIndex, int yIndex)
   {
      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         return Double.NaN;
      }

      return heightMap[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)];
   }

   public double getMinHeight()
   {
      double minValue = Double.POSITIVE_INFINITY;
      for (int i = 0; i < heightMap.length; i++)
      {
         if (!Double.isNaN(heightMap[i]) && heightMap[i] < minValue)
            minValue = heightMap[i];
      }

      return minValue;
   }

   /**
    * Returns a traversability score from 0 to 1, where 0 is non-traversable and 1 is perfectly flat and level terrain.
    */
   public double getTraversabilityScore(double x, double y)
   {
      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenterX, cellSize, centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenterY, cellSize, centerIndex);
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, xIndex, yIndex))
         return Double.NaN;

      int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return traversabilityScoreMap[key];
   }

   public SnapResult getTraversabilityClass(double x, double y)
   {
      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenterX, cellSize, centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenterY, cellSize, centerIndex);
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, xIndex, yIndex))
         return null;

      int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return SnapResult.values[traversabilityClassMap[key]];
   }

   public UnitVector3DReadOnly getNormal(double x, double y)
   {
      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenterX, cellSize, centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenterY, cellSize, centerIndex);
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, xIndex, yIndex))
         return Axis3D.Z;

      int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return new UnitVector3D(unpackByteAsFloat(snapNormalXMap, key, -1.0f, 1.0f),
                              unpackByteAsFloat(snapNormalYMap, key, -1.0f, 1.0f),
                              unpackByteAsFloat(snapNormalZMap, key, 0.0f, 1.0f));
   }

   public float[] getHeightMap()
   {
      return heightMap;
   }

   public float[] getTraversabilityScoreMap()
   {
      return traversabilityScoreMap;
   }

   public byte[] getTraversabilityClassMap()
   {
      return traversabilityClassMap;
   }

   public byte[] getSnapNormalXMap()
   {
      return snapNormalXMap;
   }

   public byte[] getSnapNormalYMap()
   {
      return snapNormalYMap;
   }

   public byte[] getSnapNormalZMap()
   {
      return snapNormalZMap;
   }

   public void setHeightMap(float[] heightMap)
   {
      System.arraycopy(heightMap, 0, this.heightMap, 0, this.heightMap.length);
   }

   public void setTraversabilityScoreMap(float[] traversabilityScoreMap)
   {
      System.arraycopy(traversabilityScoreMap, 0, this.traversabilityScoreMap, 0, this.traversabilityScoreMap.length);
   }

   public void setTraversabilityClassMap(byte[] traversabilityClassMap)
   {
      System.arraycopy(traversabilityClassMap, 0, this.traversabilityClassMap, 0, this.traversabilityClassMap.length);
   }

   public void setSnapNormalXMap(byte[] snapNormalXMap)
   {
      System.arraycopy(snapNormalXMap, 0, this.snapNormalXMap, 0, this.snapNormalXMap.length);
   }

   public void setSnapNormalYMap(byte[] snapNormalYMap)
   {
      System.arraycopy(snapNormalYMap, 0, this.snapNormalYMap, 0, this.snapNormalYMap.length);
   }

   public void setSnapNormalZMap(byte[] snapNormalZMap)
   {
      System.arraycopy(snapNormalZMap, 0, this.snapNormalZMap, 0, this.snapNormalZMap.length);
   }

   public static float unpackByteAsFloat(byte[] byteArray, int index, float minValue, float maxValue)
   {
      return unpackByteAsFloat(byteArray[index], minValue, maxValue);
   }

   public static float unpackByteAsFloat(byte val, float minValue, float maxValue)
   {
      return (float) (val & 0xFF) * (maxValue - minValue) / 255 + minValue;
   }

   public static byte packFloatAsByte(float value, float minValue, float maxValue)
   {
      int val =  ((byte) ((value - minValue) * 255 / (maxValue - minValue))) & 0xFF;
      return (byte) val;
   }


   public void setGridCenterX(double gridCenterX)
   {
      this.gridCenterX = gridCenterX;
   }

   public void setGridCenterY(double gridCenterY)
   {
      this.gridCenterY = gridCenterY;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public double getMapSize()
   {
      return mapSize;
   }

   public double getCellSize()
   {
      return cellSize;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public double getGridCenterX()
   {
      return gridCenterX;
   }

   public double getGridCenterY()
   {
      return gridCenterY;
   }
}

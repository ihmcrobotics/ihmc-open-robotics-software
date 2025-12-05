package us.ihmc.perception.gpuMapping;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

import java.util.Arrays;

public class TerrainMapData
{
   private final double cellSize;
   private final double mapSize;
   private double gridCenterX;
   private double gridCenterY;
   private final int centerIndex;
   private final int cellsPerAxis;

   private float[] heightMap;

   private float[] traversabilityScoreMap;
   private byte[] traversabilityClassMap;

   private byte[] snapNormalXMap;
   private byte[] snapNormalYMap;
   private byte[] snapNormalZMap;

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

   public void setHeight(double x, double y, double z)
   {
      int key = HeightMapTools.coordinateToKey(x, y, gridCenterX, gridCenterY, cellSize, centerIndex);
      heightMap[key] = (float) z;
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
      return SnapResult.values()[traversabilityClassMap[key]];
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
      this.heightMap = heightMap;
   }

   public void setTraversabilityScoreMap(float[] traversabilityScoreMap)
   {
      this.traversabilityScoreMap = traversabilityScoreMap;
   }

   public void setTraversabilityClassMap(byte[] traversabilityClassMap)
   {
      this.traversabilityClassMap = traversabilityClassMap;
   }

   public void setSnapNormalXMap(byte[] snapNormalXMap)
   {
      this.snapNormalXMap = snapNormalXMap;
   }

   public void setSnapNormalYMap(byte[] snapNormalYMap)
   {
      this.snapNormalYMap = snapNormalYMap;
   }

   public void setSnapNormalZMap(byte[] snapNormalZMap)
   {
      this.snapNormalZMap = snapNormalZMap;
   }

   public static float unpackByteAsFloat(byte[] byteArray, int index, float minValue, float maxValue)
   {
      return (float) (byteArray[index] & 0xFF) * (maxValue - minValue) / 255 + minValue;
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

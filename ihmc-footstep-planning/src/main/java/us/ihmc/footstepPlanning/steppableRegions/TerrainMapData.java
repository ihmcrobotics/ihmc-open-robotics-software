package us.ihmc.footstepPlanning.steppableRegions;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.util.Arrays;

public class TerrainMapData
{
   private HeightMapData heightMapData;

   private float[] traversabilityScoreMap;
   private byte[] traversabilityClassMap;

   private byte[] snapNormalXMap;
   private byte[] snapNormalYMap;
   private byte[] snapNormalZMap;

   public TerrainMapData(int cellsPerAxis)
   {
      traversabilityScoreMap = new float[cellsPerAxis * cellsPerAxis];
      traversabilityClassMap = new byte[cellsPerAxis * cellsPerAxis];

      snapNormalXMap = new byte[cellsPerAxis * cellsPerAxis];
      snapNormalYMap = new byte[cellsPerAxis * cellsPerAxis];
      snapNormalZMap = new byte[cellsPerAxis * cellsPerAxis];
   }

   public TerrainMapData(TerrainMapData other)
   {
      this.heightMapData = new HeightMapData(other.heightMapData);

      int size = heightMapData.getCellsPerAxis() * heightMapData.getCellsPerAxis();

      this.traversabilityScoreMap = Arrays.copyOf(other.traversabilityScoreMap, size);
      this.traversabilityClassMap = Arrays.copyOf(other.traversabilityClassMap, size);

      this.snapNormalXMap = Arrays.copyOf(other.snapNormalXMap, size);
      this.snapNormalYMap = Arrays.copyOf(other.snapNormalYMap, size);
      this.snapNormalZMap = Arrays.copyOf(other.snapNormalZMap, size);
   }

   public HeightMapData getHeightMapData()
   {
      return heightMapData;
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public double getHeightInWorld(double x, double y)
   {
      return heightMapData.getHeight(x, y);
   }

   /**
    * Returns a traversability score from 0 to 1, where 0 is non-traversable and 1 is perfectly flat and level terrain.
    */
   public double getTraversabilityScore(double x, double y)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapData.getMapSize(), heightMapData.getCellSize());
      int xIndex = HeightMapTools.coordinateToIndex(x, heightMapData.getGridCenter().getX(), heightMapData.getCellSize(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, heightMapData.getGridCenter().getY(), heightMapData.getCellSize(), centerIndex);
      if (TerrainMapTools.isOutOfBounds(heightMapData.getCellsPerAxis(), xIndex, yIndex))
         return Double.NaN;

      int index = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return traversabilityScoreMap[index];
   }

   public SnapResult getTraversabilityClass(double x, double y)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapData.getMapSize(), heightMapData.getCellSize());
      int xIndex = HeightMapTools.coordinateToIndex(x, heightMapData.getGridCenter().getX(), heightMapData.getCellSize(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, heightMapData.getGridCenter().getY(), heightMapData.getCellSize(), centerIndex);
      if (TerrainMapTools.isOutOfBounds(heightMapData.getCellsPerAxis(), xIndex, yIndex))
         return null;

      int index = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return SnapResult.values()[traversabilityClassMap[index]];
   }

   public UnitVector3DReadOnly getNormal(double x, double y)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapData.getMapSize(), heightMapData.getCellSize());
      int xIndex = HeightMapTools.coordinateToIndex(x, heightMapData.getGridCenter().getX(), heightMapData.getCellSize(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, heightMapData.getGridCenter().getY(), heightMapData.getCellSize(), centerIndex);
      if (TerrainMapTools.isOutOfBounds(heightMapData.getCellsPerAxis(), xIndex, yIndex))
         return Axis3D.Z;

      int index = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      return new UnitVector3D(unpackByteAsFloat(snapNormalXMap, index, -1.0f, 1.0f),
                              unpackByteAsFloat(snapNormalXMap, index, -1.0f, 1.0f),
                              unpackByteAsFloat(snapNormalXMap, index, 0.0f, 1.0f));
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
}

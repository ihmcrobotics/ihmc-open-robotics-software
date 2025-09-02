package us.ihmc.footstepPlanning.steppableRegions;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.util.Arrays;

public class TerrainMapData
{
   /**
    * Sensor origin that defines the center of the height map
    */
   private Point2D terrainMapCenter = new Point2D();

   private final int cellsPerAxis;

   private final double gridResolutionXY;
   private final double gridSizeXY;

   private float[] heightMap;

   private byte[] terrainCostMap;
   private byte[] contactMap;

   private byte[] snapNormalXMap;
   private byte[] snapNormalYMap;
   private byte[] snapNormalZMap;

   private byte[] snappedAreaFractionMap;
   private byte[] steppabilityMap;
   private byte[] steppabilityConnectionsMap;

   private int centerIndex;

   public TerrainMapData(int cellsPerAxis, double gridResolutionXY, double gridSizeXY)
   {
      this.cellsPerAxis = cellsPerAxis;
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);

      heightMap = new float[cellsPerAxis * cellsPerAxis];
      terrainCostMap = new byte[cellsPerAxis * cellsPerAxis];
      contactMap = new byte[cellsPerAxis * cellsPerAxis];

      snapNormalXMap = new byte[cellsPerAxis * cellsPerAxis];
      snapNormalYMap = new byte[cellsPerAxis * cellsPerAxis];
      snapNormalZMap = new byte[cellsPerAxis * cellsPerAxis];

      snappedAreaFractionMap = new byte[cellsPerAxis * cellsPerAxis];
      steppabilityMap = new byte[cellsPerAxis * cellsPerAxis];
      steppabilityConnectionsMap = new byte[cellsPerAxis * cellsPerAxis];
   }

   public TerrainMapData(TerrainMapData other)
   {
      this.cellsPerAxis = other.cellsPerAxis;
      this.gridResolutionXY = other.gridResolutionXY;
      this.gridSizeXY = other.gridSizeXY;
      this.centerIndex = other.centerIndex;

      this.terrainMapCenter = new Point2D(other.terrainMapCenter);
      int size = cellsPerAxis * cellsPerAxis;

      this.heightMap = Arrays.copyOf(other.heightMap, size);
      this.terrainCostMap = Arrays.copyOf(other.terrainCostMap, size);
      this.contactMap = Arrays.copyOf(other.contactMap, size);

      this.snapNormalXMap = Arrays.copyOf(other.snapNormalXMap, size);
      this.snapNormalYMap = Arrays.copyOf(other.snapNormalYMap, size);
      this.snapNormalZMap = Arrays.copyOf(other.snapNormalZMap, size);

      this.snappedAreaFractionMap = Arrays.copyOf(other.snappedAreaFractionMap, size);
      this.steppabilityMap = Arrays.copyOf(other.steppabilityMap, size);
      this.steppabilityConnectionsMap = Arrays.copyOf(other.steppabilityConnectionsMap, size);
   }

   public int getLocalXIndex(double coordinate)
   {
      return getLocalIndex(coordinate, terrainMapCenter.getX());
   }

   public int getLocalYIndex(double coordinate)
   {
      return getLocalIndex(coordinate, terrainMapCenter.getY());
   }

   private int getLocalIndex(double coordinate, double center)
   {
      int cellsPerMeter = (int) (1.0 / gridResolutionXY);
      return TerrainMapTools.getLocalIndex(cellsPerMeter, cellsPerAxis, coordinate, center);
   }

   public float getSnappedAreaFractionInWorld(double x, double y)
   {
      int rIndex = getLocalXIndex(x);
      int cIndex = getLocalYIndex(y);
      return getSnappedAreaLocal(rIndex, cIndex);
   }

   public float getHeightInWorld(double x, double y)
   {
      int rIndex = getLocalXIndex(x);
      int cIndex = getLocalYIndex(y);
      return getHeightFloatLocal(rIndex, cIndex);
   }

   public float getContactScoreInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, terrainMapCenter.getX32());
      int cIndex = getLocalIndex(y, terrainMapCenter.getY32());
      return getContactScoreLocal(rIndex, cIndex);
   }

   public float getSteppabilityInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, terrainMapCenter.getX());
      int cIndex = getLocalIndex(y, terrainMapCenter.getY());
      return getSteppabilityLocal(rIndex, cIndex);
   }

   public UnitVector3DReadOnly getNormalInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, terrainMapCenter.getX32());
      int cIndex = getLocalIndex(y, terrainMapCenter.getY32());
      return getNormalLocal(rIndex, cIndex);
   }

   public SnapResult getSnapResultInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, terrainMapCenter.getX32());
      int cIndex = getLocalIndex(y, terrainMapCenter.getY32());
      return getSnapResultLocal(rIndex, cIndex);
   }

   public float getHeightFloatLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, rIndex, cIndex))
         return 0.0f;

      return heightMap[rIndex * cellsPerAxis + cIndex];
   }

   public float getSnappedAreaLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, rIndex, cIndex))
         return 0.0f;

      // This mask is necessary because the area is stored as an unsigned char, and it discards all the additional information past that one byte. We then scale
      // by 255, which is the maximum value that the char can contain, to convert the output to be between 0 and 1.
      return ((float) ((snappedAreaFractionMap[rIndex * cellsPerAxis + cIndex] & 0xFF))) / 255;
   }

   public UnitVector3DReadOnly getNormalLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, rIndex, cIndex))
      {
         return new UnitVector3D(0.0, 0.0, 1.0);
      }

      return new UnitVector3D(getNormalLocalUnsafe(snapNormalXMap, rIndex, cIndex),
                              getNormalLocalUnsafe(snapNormalYMap, rIndex, cIndex),
                              getNormalLocalUnsafe(snapNormalZMap, rIndex, cIndex));
   }

   private float getNormalLocalUnsafe(byte[] normalArray, int rIndex, int cIndex)
   {
      // This mask is necessary because the norm is stored as an unsigned short, and it discards all the additional information past that one byte. We then
      // scale by 2 / 255, to bring it in-range of the whole value
      return (float) ((normalArray[rIndex * cellsPerAxis + cIndex] & 0xFF)) * 2 / 255 - 1.0f;
   }

   private float getContactScoreLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, rIndex, cIndex))
         return 0.0f;

      return (float) (contactMap[rIndex * cellsPerAxis + cIndex] & 0xFF);
   }

   private SnapResult getSnapResultLocal(int rIndex, int cIndex)
   {
      return SnapResult.fromByte(getSteppabilityLocal(rIndex, cIndex));
   }

   private int getSteppabilityLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, rIndex, cIndex))
         return SnapResult.SNAP_FAILED.ordinal();

      // This mask is necessary because the area is stored as an unsigned char, and it discards all the additional information past that one byte.
      return steppabilityMap[rIndex * cellsPerAxis + cIndex] & 0xFF;
   }

   public void setHeightFloatLocal(float height, int rIndex, int cIndex)
   {
      heightMap[rIndex * cellsPerAxis + cIndex] = height;
   }

   public void setSensorOrigin(Tuple3DReadOnly origin)
   {
      setSensorOrigin(origin.getX(), origin.getY());
   }

   public void setSensorOrigin(Tuple2DReadOnly origin)
   {
      setSensorOrigin(origin.getX(), origin.getY());
   }

   public void setSensorOrigin(double originX, double originY)
   {
      this.terrainMapCenter.set(originX, originY);
   }

   public Point2DReadOnly getTerrainMapCenter()
   {
      return terrainMapCenter;
   }

   public float[] getHeightMap()
   {
      return heightMap;
   }

   public byte[] getTerrainCostMap()
   {
      return terrainCostMap;
   }

   public byte[] getContactMap()
   {
      return contactMap;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public double getGridResolutionXY()
   {
      return gridResolutionXY;
   }

   public double getGridSizeXY()
   {
      return gridSizeXY;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public void setTerrainMapCenter(Point2DReadOnly terrainMapCenter)
   {
      this.terrainMapCenter = new Point2D(terrainMapCenter);
   }

   public void setHeightMap(float[] heightMap)
   {
      this.heightMap = heightMap;
   }

   public void setTerrainCostMap(byte[] terrainCostMap)
   {
      this.terrainCostMap = terrainCostMap;
   }

   public void setContactMap(byte[] contactMap)
   {
      this.contactMap = contactMap;
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

   public void setSnappedAreaFractionMap(byte[] snappedAreaFractionMap)
   {
      this.snappedAreaFractionMap = snappedAreaFractionMap;
   }

   public void setSteppabilityMap(byte[] steppabilityMap)
   {
      this.steppabilityMap = steppabilityMap;
   }

   public void setSteppabilityConnectionsMap(byte[] steppabilityConnectionsMap)
   {
      this.steppabilityConnectionsMap = steppabilityConnectionsMap;
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

   public byte[] getSnappedAreaFractionMap()
   {
      return snappedAreaFractionMap;
   }

   public byte[] getSteppabilityMap()
   {
      return steppabilityMap;
   }

   public byte[] getSteppabilityConnectionsMap()
   {
      return steppabilityConnectionsMap;
   }
}

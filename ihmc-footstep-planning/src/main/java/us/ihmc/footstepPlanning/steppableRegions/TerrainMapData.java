package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.opencv.OpenCVTools;

import java.nio.ByteBuffer;

public class TerrainMapData
{
   /**
    * Sensor origin that defines the center of the height map
    */
   private final Point2D terrainMapCenter = new Point2D();

   private int cellsPerAxis;

   private double gridResolutionXY;
   private double gridSizeXY;

   private float[] heightMap;

   private byte[] terrainCostMap;
   private byte[] contactMap;

   private byte[] snapNormalXMap;
   private byte[] snapNormalYMap;
   private byte[] snapNormalZMap;

   private Mat steppabilityImage;
   private Mat steppabilityConnectionsMat;
   private Mat snappedAreaFractionImage;
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
   }

   public TerrainMapData(TerrainMapData other)
   {
      this.cellsPerAxis = other.cellsPerAxis;
      this.gridResolutionXY = other.gridResolutionXY;
      this.centerIndex = other.centerIndex;

      terrainMapCenter.set(other.terrainMapCenter);

      heightMap = other.heightMap;
      terrainCostMap = other.terrainCostMap;
      contactMap = other.contactMap;

      snapNormalXMap = other.snapNormalXMap;
      snapNormalYMap = other.snapNormalYMap;
      snapNormalZMap = other.snapNormalZMap;

      setSteppabilityMat(other.steppabilityImage);
      setSteppabilityConnectionsMat(other.steppabilityConnectionsMat);
      setSnappedAreaFractionMat(other.snappedAreaFractionImage);
   }

   public TerrainMapData(TerrainMapMessage other)
   {
      setFromPacket(other);
   }

   public boolean isEmpty()
   {
      if (hasHeightMapFloats())
         return false;
      return !hasSteppability();
   }

   public boolean hasTerrainCostArray()
   {
      return terrainCostMap != null;
   }

   public boolean hasCostArray()
   {
      return contactMap != null;
   }

   public boolean hasHeightMapFloats()
   {
      return heightMap != null;
   }

   public boolean hasSnapNormal()
   {
      return snapNormalXMap != null && snapNormalYMap != null && snapNormalZMap != null;
   }

   public boolean hasSteppability()
   {
      return steppabilityImage != null;
   }

   public boolean hasSteppableConnections()
   {
      return steppabilityConnectionsMat != null;
   }

   public boolean hasSnappedArea()
   {
      return snappedAreaFractionImage != null;
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
      return ((float) ((snappedAreaFractionImage.ptr(rIndex, cIndex).get() & 0xFF))) / 255;
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
      return steppabilityImage.ptr(rIndex, cIndex).get() & 0xFF;
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

   public void setSteppabilityMat(Mat steppabilityImage)
   {
      this.steppabilityImage = steppabilityImage == null ? null : steppabilityImage.clone();
   }

   public void setSteppabilityConnectionsMat(Mat steppabilityConnectionsImage)
   {
      this.steppabilityConnectionsMat = steppabilityConnectionsImage == null ? null : steppabilityConnectionsImage.clone();
   }

   public void setSnappedAreaFractionMat(Mat snappedAreaFractionImage)
   {
      this.snappedAreaFractionImage = snappedAreaFractionImage == null ? null : snappedAreaFractionImage.clone();
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

   public Mat getSteppabilityMat()
   {
      return steppabilityImage;
   }

   public Mat getSteppabilityConnectionsMat()
   {
      return steppabilityConnectionsMat;
   }

   public Mat getSnappedAreaFractionMat()
   {
      return snappedAreaFractionImage;
   }

   public void setFromPacket(TerrainMapMessage message)
   {
      cellsPerAxis = message.getLocalGridSize();

      terrainMapCenter.set(message.getMapCenterX(), message.getMapCenterY());

      if (message.getHasTerrainCostData())
      {
         if (terrainCostMap == null)
            terrainCostMap = new byte[cellsPerAxis * cellsPerAxis];

         message.getTerrainCostData().add(terrainCostMap);
      }
      else
      {
         terrainCostMap = null;
      }
      if (message.getHasContactMapData())
      {
         if (contactMap == null)
            contactMap = new byte[cellsPerAxis * cellsPerAxis];

         message.getContactMapData().add(contactMap);
      }
      else
      {
         contactMap = null;
      }
      if (message.getHasHeightMapFloatData())
      {
         if (heightMap == null)
            heightMap = new float[cellsPerAxis * cellsPerAxis];

         message.getHeights().add(heightMap);
      }
      else
      {
         heightMap = null;
      }
      if (message.getHasSnappedNormalData())
      {
         if (snapNormalXMap == null)
            snapNormalXMap = new byte[cellsPerAxis * cellsPerAxis];
         message.getSnappedNormalXData().add(snapNormalXMap);
         if (snapNormalYMap == null)
            snapNormalYMap = new byte[cellsPerAxis * cellsPerAxis];
         message.getSnappedNormalYData().add(snapNormalYMap);
         if (snapNormalZMap == null)
            snapNormalZMap = new byte[cellsPerAxis * cellsPerAxis];
         message.getSnappedNormalZData().add(snapNormalZMap);
      }
      else
      {
         snapNormalXMap = null;
         snapNormalYMap = null;
         snapNormalZMap = null;
      }
      if (message.getHasSteppabilityData())
      {
         if (steppabilityImage == null)
            steppabilityImage = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getSteppabilityData().getBuffer(), steppabilityImage);
      }
      else
      {
         steppabilityImage = null;
      }
      if (message.getHasSteppableConnectionsData())
      {
         if (steppabilityConnectionsMat == null)
            steppabilityConnectionsMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getSteppableConnectionsData().getBuffer(), steppabilityConnectionsMat);
      }
      else
      {
         steppabilityConnectionsMat = null;
      }
      if (message.getHasSnappedAreaData())
      {
         if (snappedAreaFractionImage == null)
            snappedAreaFractionImage = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getSnappedAreaData().getBuffer(), snappedAreaFractionImage);
      }
      else
      {
         snappedAreaFractionImage = null;
      }
   }

   private void packDataIntoMatFromByteBuffer(ByteBuffer buffer, Mat dataToPack)
   {
      dataToPack.data().put(buffer.array(), 0, (int) OpenCVTools.dataSize(dataToPack));
   }
}

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
   private int cellsPerMeter = 50;

   private double gridResolutionXY;
   private double gridSizeXY;

   private float[] heightMapFloats;
   private Mat terrainCostMap;
   private Mat contactMap;

   private Mat snapNormalXImage;
   private Mat snapNormalYImage;
   private Mat snapNormalZImage;
   private Mat steppabilityImage;
   private Mat steppabilityConnectionsMat;
   private Mat snappedAreaFractionImage;
   private int centerIndex;

   public TerrainMapData(int cellsPerAxis, double gridResolutionXY, double gridSizeXY)
   {
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;

      centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);

      heightMapFloats = new float[cellsPerAxis * cellsPerAxis];
      this.cellsPerAxis = cellsPerAxis;
   }

   public TerrainMapData(TerrainMapData other)
   {
      this.cellsPerAxis = other.cellsPerAxis;
      this.cellsPerMeter = other.cellsPerMeter;
      this.gridResolutionXY = other.gridResolutionXY;
      this.centerIndex = other.centerIndex;

      terrainMapCenter.set(other.terrainMapCenter);

      heightMapFloats  = other.heightMapFloats;
      setTerrainCostMap(other.terrainCostMap);
      setContactMap(other.contactMap);
      setSnapNormalXMat(other.snapNormalXImage);
      setSnapNormalYMat(other.snapNormalYImage);
      setSnapNormalZMat(other.snapNormalZImage);
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

   public boolean hasTerrainCost()
   {
      return terrainCostMap != null;
   }

   public boolean hasContactMap()
   {
      return contactMap != null;
   }

   public boolean hasHeightMapFloats()
   {
      return heightMapFloats != null;
   }

   public boolean hasSnapNormal()
   {
      return snapNormalXImage != null && snapNormalYImage != null && snapNormalZImage != null;
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

      return heightMapFloats[rIndex * cellsPerAxis + cIndex];
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

      return new UnitVector3D(getNormalLocalUnsafe(snapNormalXImage, rIndex, cIndex),
                              getNormalLocalUnsafe(snapNormalYImage, rIndex, cIndex),
                              getNormalLocalUnsafe(snapNormalZImage, rIndex, cIndex));
   }

   private static float getNormalLocalUnsafe(Mat normalImage, int rIndex, int cIndex)
   {
      // This mask is necessary because the norm is stored as an unsigned short, and it discards all the additional information past that one byte. We then
      // scale by 2 / 255, to bring it in-range of the whole value
      return ((float) ((normalImage.ptr(rIndex, cIndex).get() & 0xFF))) * 2 / 255 - 1.0f;
   }

   private float getContactScoreLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(cellsPerAxis, rIndex, cIndex))
         return 0.0f;

      return (float) ((contactMap.ptr(rIndex, cIndex).get() & 0xFF));
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
      heightMapFloats[rIndex * cellsPerAxis + cIndex] = height;
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

   public float[] getHeightMapFloats()
   {
      return heightMapFloats;
   }

   public Mat getTerrainCostMap()
   {
      return terrainCostMap;
   }

   public Mat getContactMap()
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

   public void setTerrainCostMap(Mat terrainCostMap)
   {
      this.terrainCostMap = terrainCostMap == null ? null : terrainCostMap.clone();
   }

   public void setHeightMapFloats(float[] heightMapFloats)
   {
      this.heightMapFloats = heightMapFloats;
   }

   public void setContactMap(Mat contactMap)
   {
      this.contactMap = contactMap == null ? null : contactMap.clone();
   }


   public void setSnapNormalXMat(Mat snapNormalXImage)
   {
      this.snapNormalXImage = snapNormalXImage == null ? null : snapNormalXImage.clone();
   }

   public void setSnapNormalYMat(Mat snapNormalYImage)
   {
      this.snapNormalYImage = snapNormalYImage == null ? null : snapNormalYImage.clone();
   }

   public void setSnapNormalZMat(Mat snapNormalZImage)
   {
      this.snapNormalZImage = snapNormalZImage == null ? null : snapNormalZImage.clone();
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

   public Mat getSnapNormalXMat()
   {
      return snapNormalXImage;
   }

   public Mat getSnapNormalYMat()
   {
      return snapNormalYImage;
   }

   public Mat getSnapNormalZMat()
   {
      return snapNormalZImage;
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
      cellsPerMeter = message.getCellsPerMeter();

      terrainMapCenter.set(message.getMapCenterX(), message.getMapCenterY());

      if (message.getHasTerrainCostData())
      {
         if (terrainCostMap == null)
            terrainCostMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getTerrainCostData().getBuffer(), terrainCostMap);
      }
      else
      {
         terrainCostMap = null;
      }
      if (message.getHasContactMapData())
      {
         if (contactMap == null)
            contactMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getContactMapData().getBuffer(), contactMap);
      }
      else
      {
         contactMap = null;
      }
      if (message.getHasHeightMapFloatData())
      {
         if (heightMapFloats == null)
            heightMapFloats = new float[cellsPerAxis * cellsPerAxis];

         message.getHeights().add(heightMapFloats);
      }
      else
      {
         heightMapFloats = null;
      }
      if (message.getHasSnappedNormalData())
      {
         if (snapNormalXImage == null)
            snapNormalXImage = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getSnappedNormalXData().getBuffer(), snapNormalXImage);
         if (snapNormalYImage == null)
            snapNormalYImage = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getSnappedNormalYData().getBuffer(), snapNormalYImage);
         if (snapNormalZImage == null)
            snapNormalZImage = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
         packDataIntoMatFromByteBuffer(message.getSnappedNormalZData().getBuffer(), snapNormalZImage);
      }
      else
      {
         snapNormalXImage = null;
         snapNormalYImage = null;
         snapNormalZImage = null;
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

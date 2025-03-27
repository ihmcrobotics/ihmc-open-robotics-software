package us.ihmc.perception.heightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.perception.steppableRegions.SnapResult;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class TerrainMapData
{
   /**
    * Sensor origin that defines the center of the height map
    */
   private final Point2D terrainMapCenter = new Point2D();

   private int localGridSize;
   private int cellsPerMeter = 50;

   private Mat heightMap;
   // TODO the contact map and terrain map are assumed to be null. I'm keeping them in to avoid breaking the code.
   private Mat contactMap;

   private Mat steppabilityImage;
   private Mat snapHeightImage;
   private Mat snappedAreaFractionImage;
   private Mat snapNormalXImage;
   private Mat snapNormalYImage;
   private Mat snapNormalZImage;

   private double heightScaleFactor;
   private double heightScaleOffset;

   public TerrainMapData(int height, int width, HeightMapParameters heightMapParameters)
   {
      setHeightScaleParameters(heightMapParameters.getHeightScaleFactor(), heightMapParameters.getHeightOffset());

      heightMap = new Mat(height, width, opencv_core.CV_16UC1);

//      contactMap = new Mat(height, width, opencv_core.CV_8UC1);
      localGridSize = height;
   }

   // FIXME this copy constructor probably has a number of problems with it.
   public TerrainMapData(TerrainMapData other)
   {
      this.localGridSize = other.localGridSize;
      this.cellsPerMeter = other.cellsPerMeter;
      this.heightScaleFactor = other.heightScaleFactor;
      this.heightScaleOffset = other.heightScaleOffset;

      terrainMapCenter.set(other.terrainMapCenter);

      setHeightMap(other.heightMap);

      setSteppabilityImage(other.steppabilityImage);
      setSnapHeightImage(other.snapHeightImage);
      setSnappedAreaFractionImage(other.snappedAreaFractionImage);
      setSnapNormalXImage(other.snapNormalXImage);
      setSnapNormalYMat(other.snapNormalYImage);
      setSnapNormalZImage(other.snapNormalZImage);
   }

   public boolean hasHeightMap()
   {
      return heightMap != null;
   }

   public boolean hasSnapHeight()
   {
      return snapHeightImage != null;
   }

   public boolean hasSnapNormal()
   {
      return snapNormalXImage != null && snapNormalYImage != null && snapNormalZImage != null;
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
      return TerrainMapTools.getLocalIndex(cellsPerMeter, localGridSize, coordinate, center);
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
      return getHeightLocal(rIndex, cIndex);
   }

   public float getSnappedHeightInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, terrainMapCenter.getX());
      int cIndex = getLocalIndex(y, terrainMapCenter.getY());
      return getSnappedHeightLocal(rIndex, cIndex);
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

   public float getHeightLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(localGridSize, rIndex, cIndex))
         return 0.0f;

      // This mask is necessary because the height is stored as a short, and it discards all the additional information past those two bytes.
      float height = ((int) heightMap.ptr(rIndex, cIndex).getShort() & 0xFFFF);
      return TerrainMapTools.convertScaledAndOffsetValue((float) heightScaleFactor, (float) heightScaleOffset, height);
   }

   public float getSnappedAreaLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(localGridSize, rIndex, cIndex))
         return 0.0f;

      // This mask is necessary because the area is stored as an unsigned char, and it discards all the additional information past that one byte. We then scale
      // by 255, which is the maximum value that the char can contain, to convert the output to be between 0 and 1.
      return ((float) ((snappedAreaFractionImage.ptr(rIndex, cIndex).get() & 0xFF))) / 255;
   }

   public float getSnappedHeightLocal(int rIndex, int cIndex)
   {
      if (snapHeightImage == null)
         return getHeightLocal(rIndex, cIndex);

      if (TerrainMapTools.isOutOfBounds(localGridSize, rIndex, cIndex))
         return 0.0f;

      // This mask is necessary because the height is stored as a short, and it discards all the additional information past those two bytes.
      int height = ((int) snapHeightImage.ptr(rIndex, cIndex).getShort() & 0xFFFF);
      return TerrainMapTools.convertScaledAndOffsetValue((float) heightScaleFactor, (float) heightScaleOffset, height);
   }

   public UnitVector3DReadOnly getNormalLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(localGridSize, rIndex, cIndex))
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
      if (TerrainMapTools.isOutOfBounds(localGridSize, rIndex, cIndex))
         return 0.0f;

      return (float) ((contactMap.ptr(rIndex, cIndex).get() & 0xFF));
   }

   private SnapResult getSnapResultLocal(int rIndex, int cIndex)
   {
      return SnapResult.fromByte(getSteppabilityLocal(rIndex, cIndex));
   }

   private int getSteppabilityLocal(int rIndex, int cIndex)
   {
      if (TerrainMapTools.isOutOfBounds(localGridSize, rIndex, cIndex))
         return SnapResult.SNAP_FAILED.ordinal();

      // This mask is necessary because the area is stored as an unsigned char, and it discards all the additional information past that one byte.
      return steppabilityImage.ptr(rIndex, cIndex).get() & 0xFF;
   }

   public void setHeightLocal(float height, int rIndex, int cIndex)
   {
      float offsetHeight = height + (float) heightScaleOffset;
      int finalHeight = (int) (offsetHeight * heightScaleFactor);
      heightMap.ptr(rIndex, cIndex).putShort((short) finalHeight);
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

   public Mat getHeightMap()
   {
      return heightMap;
   }

   public Mat getContactMap()
   {
      return contactMap;
   }

   public int getCellsPerMeter()
   {
      return cellsPerMeter;
   }

   public int getLocalGridSize()
   {
      return localGridSize;
   }

   public void setHeightScaleParameters(double heightScaleFactor, double heightScaleOffset)
   {
      this.heightScaleOffset = heightScaleOffset;
      this.heightScaleFactor = heightScaleFactor;
   }

   public void setHeightMap(Mat heightMap)
   {
      this.heightMap = heightMap == null ? null : heightMap.clone();
   }

   public void setContactMap(Mat contactMap)
   {
      this.contactMap = contactMap == null ? null : contactMap.clone();
   }

   public void setSteppabilityImage(Mat steppabilityImage)
   {
      this.steppabilityImage = steppabilityImage == null ? null : steppabilityImage.clone();
   }

   public void setSnapHeightImage(Mat snapHeightImage)
   {
      this.snapHeightImage = snapHeightImage == null ? null : snapHeightImage.clone();
   }

   public void setSnapNormalXImage(Mat snapNormalXImage)
   {
      this.snapNormalXImage = snapNormalXImage == null ? null : snapNormalXImage.clone();
   }

   public void setSnapNormalYMat(Mat snapNormalYImage)
   {
      this.snapNormalYImage = snapNormalYImage == null ? null : snapNormalYImage.clone();
   }

   public void setSnapNormalZImage(Mat snapNormalZImage)
   {
      this.snapNormalZImage = snapNormalZImage == null ? null : snapNormalZImage.clone();
   }

   public void setSnappedAreaFractionImage(Mat snappedAreaFractionImage)
   {
      this.snappedAreaFractionImage = snappedAreaFractionImage == null ? null : snappedAreaFractionImage.clone();
   }

   public Mat getSteppabilityImage()
   {
      return steppabilityImage;
   }

   public Mat getSnapHeightImage()
   {
      return snapHeightImage;
   }

   public Mat getSnappedAreaFractionImage()
   {
      return snappedAreaFractionImage;
   }

   public Mat getSnapNormalXImage()
   {
      return snapNormalXImage;
   }

   public Mat getSnapNormalYImage()
   {
      return snapNormalYImage;
   }

   public Mat getSnapNormalZImage()
   {
      return snapNormalZImage;
   }

   public void setFromPacket(TerrainMapMessage message)
   {
      localGridSize = message.getLocalGridSize();
      cellsPerMeter = message.getCellsPerMeter();

      terrainMapCenter.set(message.getMapCenterX(), message.getMapCenterY());

      heightScaleOffset = message.getHeightScaleOffset();
      heightScaleFactor = message.getHeightScaleFactor();

      if (heightMap == null)
         heightMap = new Mat(localGridSize, localGridSize, opencv_core.CV_16UC1);
      heightMap.data(new BytePointer(message.getHeightMapData().getBuffer()));
      if (snapHeightImage == null)
         snapHeightImage = new Mat(localGridSize, localGridSize, opencv_core.CV_16UC1);
      snapHeightImage.data(new BytePointer(message.getSnappedHeightData().getBuffer()));
      if (snapNormalXImage == null)
         snapNormalXImage = new Mat(localGridSize, localGridSize, opencv_core.CV_8UC1);
      snapNormalXImage.data(new BytePointer(message.getSnappedNormalXData().getBuffer()));
      if (snapNormalYImage == null)
         snapNormalYImage = new Mat(localGridSize, localGridSize, opencv_core.CV_8UC1);
      snapNormalYImage.data(new BytePointer(message.getSnappedNormalYData().getBuffer()));
      if (snapNormalZImage == null)
         snapNormalZImage = new Mat(localGridSize, localGridSize, opencv_core.CV_8UC1);
      snapNormalZImage.data(new BytePointer(message.getSnappedNormalZData().getBuffer()));
      if (snappedAreaFractionImage == null)
         snappedAreaFractionImage = new Mat(localGridSize, localGridSize, opencv_core.CV_8UC1);
      snappedAreaFractionImage.data(new BytePointer(message.getSnappedAreaData().getBuffer()));
   }

   public TerrainMapMessage getAsPacket()
   {
      TerrainMapMessage message = new TerrainMapMessage();

      message.setLocalGridSize(localGridSize);
      message.setCellsPerMeter((byte) cellsPerMeter);

      message.setMapCenterX(terrainMapCenter.getX());
      message.setMapCenterY(terrainMapCenter.getY());

      message.setHeightScaleFactor(heightScaleFactor);
      message.setHeightScaleOffset(heightScaleOffset);

      PerceptionMessageTools.packDataArray(message.getHeightMapData(), heightMap);
      PerceptionMessageTools.packDataArray(message.getSnappedHeightData(), snapHeightImage);
      PerceptionMessageTools.packDataArray(message.getSnappedNormalXData(), snapNormalXImage);
      PerceptionMessageTools.packDataArray(message.getSnappedNormalYData(), snapNormalYImage);
      PerceptionMessageTools.packDataArray(message.getSnappedAreaData(), snappedAreaFractionImage);

      return message;
   }
}

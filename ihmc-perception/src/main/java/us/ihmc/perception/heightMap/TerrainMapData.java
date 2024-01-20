package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.steppableRegions.SnapResult;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;

public class TerrainMapData
{
   private static final int defaultPatchSizeForNormalEstimation = 1;
   /**
    * Sensor origin that defines the center of the height map
    */
   private final Point2D heightMapCenter = new Point2D();

   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private int localGridSize = 201;
   private int cellsPerMeter = 50;

   private Mat heightMap;
   private Mat contactMap;
   private Mat terrainCostMap;

   private Mat steppableRegionAssignmentMat;
   private Mat steppableRegionRingMat;
   private Mat steppabilityImage;
   private Mat snapHeightImage;
   private Mat snapNormalXImage;
   private Mat snapNormalYImage;
   private Mat snapNormalZImage;
   private Mat steppabilityConnectionsImage;

   public TerrainMapData(Mat heightMap,
                         Mat snapHeightImage,
                         Mat contactMap,
                         Mat terrainCostMap,
                         Mat steppability,
                         Mat snapNormalXImage,
                         Mat snapNormalYImage,
                         Mat snapNormalZImage)
   {
      setHeightMap(heightMap);
      setSnapHeightImage(snapHeightImage);
      setContactMap(contactMap);
      setTerrainCostMap(terrainCostMap);
      setSteppabilityImage(steppability);
      setSnapNormalXImage(snapNormalXImage);
      setSnapNormalYImage(snapNormalYImage);
      setSnapNormalZImage(snapNormalZImage);

      this.localGridSize = heightMap.rows();
      // TODO need to add cells per meter
   }

   public TerrainMapData(TerrainMapData other)
   {
      set(other);
   }

   public TerrainMapData(int height, int width)
   {
      heightMap = new Mat(height, width, opencv_core.CV_16UC1);
      terrainCostMap = new Mat(height, width, opencv_core.CV_8UC1);
      contactMap = new Mat(height, width, opencv_core.CV_8UC1);
      localGridSize = height;
   }

   public void set(TerrainMapData other)
   {
      this.localGridSize = other.localGridSize;
      this.cellsPerMeter = other.cellsPerMeter;
      heightMapCenter.set(other.heightMapCenter);

      setHeightMap(other.heightMap);
      setSnapHeightImage(other.snapHeightImage);
      setContactMap(other.contactMap);
      setTerrainCostMap(other.terrainCostMap);
      setSteppabilityImage(other.steppabilityImage);
      setSnapNormalXImage(other.snapNormalXImage);
      setSnapNormalYImage(other.snapNormalYImage);
      setSnapNormalZImage(other.snapNormalZImage);
   }

   private int getLocalIndex(double coordinate, double center)
   {
      // TODO probably a height map tools method for this.
      return (int) ((coordinate - center) * cellsPerMeter + localGridSize / 2);
   }

   public float getHeightInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX());
      int cIndex = getLocalIndex(y, heightMapCenter.getY());
      return getHeightLocal(rIndex, cIndex);
   }

   public float getSnappedHeightInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX());
      int cIndex = getLocalIndex(y, heightMapCenter.getY());
      return getSnappedHeightLocal(rIndex, cIndex);
   }

   public float getContactScoreInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX32());
      int cIndex = getLocalIndex(y, heightMapCenter.getY32());
      return getContactScoreLocal(rIndex, cIndex);
   }

   public float getSteppabilityInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX());
      int cIndex = getLocalIndex(y, heightMapCenter.getY());
      return getSteppabilityLocal(rIndex, cIndex);
   }

   public UnitVector3DReadOnly getNormalInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX32());
      int cIndex = getLocalIndex(y, heightMapCenter.getY32());
      return getNormalLocal(rIndex, cIndex);
   }

   public SnapResult getSnapResultInWorld(double x, double y)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX32());
      int cIndex = getLocalIndex(y, heightMapCenter.getY32());
      return getSnapResultLocal(rIndex, cIndex);
   }

   public boolean hasSnapNormal()
   {
      return snapNormalXImage != null && snapNormalYImage != null && snapNormalZImage != null;
   }

   private boolean isOutOfBounds(int rIndex, int cIndex)
   {
      return rIndex < 0 || rIndex >= localGridSize || cIndex < 0 || cIndex >= localGridSize;
   }

   private static float convertScaledAndOffsetValue(float value)
   {
      return (float) (value / RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor())
             - (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset();
   }

   private float getHeightLocal(int rIndex, int cIndex)
   {
      if (isOutOfBounds(rIndex, cIndex))
         return 0.0f;

      int height = ((int) heightMap.ptr(rIndex, cIndex).getShort() & 0xFFFF);
      return convertScaledAndOffsetValue((float) height);
   }

   private float getSnappedHeightLocal(int rIndex, int cIndex)
   {
      if (snapHeightImage == null)
         return getHeightLocal(rIndex, cIndex);

      if (isOutOfBounds(rIndex, cIndex))
         return 0.0f;

      int height = ((int) snapHeightImage.ptr(rIndex, cIndex).getShort() & 0xFFFF);
      return convertScaledAndOffsetValue((float) height);
   }

   private UnitVector3DReadOnly getNormalLocal(int rIndex, int cIndex)
   {
      if (isOutOfBounds(rIndex, cIndex))
      {
         return new UnitVector3D(0.0, 0.0, 1.0);
      }

      return new UnitVector3D(getNormalLocalUnsafe(snapNormalXImage, rIndex, cIndex),
                              getNormalLocalUnsafe(snapNormalYImage, rIndex, cIndex),
                              getNormalLocalUnsafe(snapNormalZImage, rIndex, cIndex));
   }

   private static float getNormalLocalUnsafe(Mat normalImage, int rIndex, int cIndex)
   {
      return (float) ((normalImage.ptr(rIndex, cIndex).get() & 0xFF));
   }

   private float getContactScoreLocal(int rIndex, int cIndex)
   {
      if (isOutOfBounds(rIndex, cIndex))
         return 0.0f;

      return (float) ((contactMap.ptr(rIndex, cIndex).get() & 0xFF));
   }

   private SnapResult getSnapResultLocal(int rIndex, int cIndex)
   {
      return SnapResult.fromByte(getSteppabilityLocal(rIndex, cIndex));
   }

   private int getSteppabilityLocal(int rIndex, int cIndex)
   {
      if (isOutOfBounds(rIndex, cIndex))
         return SnapResult.SNAP_FAILED.ordinal();

      return steppabilityImage.ptr(rIndex, cIndex).get() & 0xFF;
   }

   public void setHeightLocal(float height, int rIndex, int cIndex)
   {
      float offsetHeight = height + (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset();
      int finalHeight = (int) (offsetHeight * RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor());
      heightMap.ptr(rIndex, cIndex).putShort((short) finalHeight);
   }

   public void fillHeightMap(HeightMapData heightMapData)
   {
      this.heightMapCenter.set(heightMapData.getGridCenter());
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

   public UnitVector3DReadOnly computeSurfaceNormalInWorld(double x, double y)
   {
      return computeSurfaceNormalInWorld(x, y, defaultPatchSizeForNormalEstimation);
   }

   public UnitVector3DReadOnly computeSurfaceNormalInWorld(double x, double y, int patchSize)
   {
      int rIndex = getLocalIndex(x, heightMapCenter.getX());
      int cIndex = getLocalIndex(y, heightMapCenter.getY());

      if (hasSnapNormal())
      {
         return getNormalLocal(rIndex, cIndex);
      }
      else
      {
         Plane3D bestFitPlane = new Plane3D();
         ArrayList<Point3D> points = new ArrayList<>();

         //LogTools.info("rIndex: {}, cIndex: {}, origin: {}", rIndex, cIndex, sensorOrigin);

         for (int i = -patchSize; i <= patchSize; i++)
         {
            for (int j = -patchSize; j <= patchSize; j++)
            {
               int r = rIndex + i;
               int c = cIndex + j;
               if (isOutOfBounds(r, c))
                  continue;

               float height = getHeightLocal(r, c);

               // compute full 3d point
               Point3D point = new Point3D();
               point.setX(heightMapCenter.getX() + (double) (r - localGridSize / 2) / cellsPerMeter);
               point.setY(heightMapCenter.getY() + (double) (c - localGridSize / 2) / cellsPerMeter);
               point.setZ(height);

               //LogTools.info("Point: {}", point);

               points.add(point);
            }
         }

         planeFitter.fitPlaneToPoints(points, bestFitPlane);
         return bestFitPlane.getNormal();
      }
   }

   public void setSensorOrigin(double originX, double originY)
   {
      this.heightMapCenter.set(originX, originY);
   }

   public Point2D getHeightMapCenter()
   {
      return heightMapCenter;
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

   public int getLocalGridSize()
   {
      return localGridSize;
   }

   public void setHeightMap(Mat heightMap)
   {
      this.heightMap = heightMap == null ? null : heightMap.clone();
   }

   public void setContactMap(Mat contactMap)
   {
      this.contactMap = contactMap == null ? null : contactMap.clone();
   }

   public void setTerrainCostMap(Mat terrainCostMap)
   {
      this.terrainCostMap = terrainCostMap == null ? null : terrainCostMap.clone();
   }

   public void setSteppableRegionRingMat(Mat steppableRegionRingMat)
   {
      this.steppableRegionRingMat = steppableRegionRingMat == null ? null : steppableRegionRingMat.clone();
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

   public void setSnapNormalYImage(Mat snapNormalYImage)
   {
      this.snapNormalYImage = snapNormalYImage == null ? null : snapNormalYImage.clone();
   }

   public void setSnapNormalZImage(Mat snapNormalZImage)
   {
      this.snapNormalZImage = snapNormalZImage == null ? null : snapNormalZImage.clone();
   }

   public void setSteppabilityConnectionsImage(Mat steppabilityConnectionsImage)
   {
      this.steppabilityConnectionsImage = steppabilityConnectionsImage == null ? null : steppabilityConnectionsImage.clone();
   }

   public void setSteppableRegionAssignmentMat(Mat steppableRegionAssignmentMat)
   {
      this.steppableRegionAssignmentMat = steppableRegionAssignmentMat == null ? null : steppableRegionAssignmentMat.clone();
   }

   public Mat getSteppableRegionAssignmentMat()
   {
      return steppableRegionAssignmentMat;
   }

   public Mat getSteppableRegionRingMat()
   {
      return steppableRegionRingMat;
   }

   public Mat getSteppabilityImage()
   {
      return steppabilityImage;
   }

   public Mat getSnapHeightImage()
   {
      return snapHeightImage;
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

   public Mat getSteppabilityConnectionsImage()
   {
      return steppabilityConnectionsImage;
   }
}

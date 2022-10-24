package us.ihmc.perception.gpuHeightMap;

import perception_msgs.msg.dds.HeightMapMessage;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class SimpleGPUHeightMap
{
   private final Point2D gridCenter = new Point2D();
   private double gridResolution;
   private double gridSizeXY;
   private int cellsPerSide;
   private int centerIndex;

   private final TIntArrayList occupiedCells = new TIntArrayList();

   private final RecyclingArrayList<Point3DBasics> centroids = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Vector3DBasics> normals = new RecyclingArrayList<>(Vector3D::new);
   private final TDoubleArrayList varianceDataMap = new TDoubleArrayList();
   private final TIntArrayList countDataMap = new TIntArrayList();

   private final BoundingBox2D boundingBox = new BoundingBox2D();
   private final BoundingBox2D occupiedBoundingBox = new BoundingBox2D();

   public SimpleGPUHeightMap()
   {
   }

   public SimpleGPUHeightMap(double gridResolution, double gridSizeXY, double gridCenterX, double gridCenterY)
   {
      reshape(gridResolution, gridSizeXY, gridCenterX, gridCenterY);
   }

   public void reshape(double gridResolution, double gridSizeXY, double gridCenterX, double gridCenterY)
   {
      this.gridResolution = gridResolution;
      this.gridSizeXY = gridSizeXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolution);
      this.cellsPerSide = 2 * centerIndex + 1;
      gridCenter.set(gridCenterX, gridCenterY);

      double epsilon = 1e-8;
      double halfWidth = 0.5 * (gridSizeXY + gridResolution) - epsilon;
      double minX = gridCenterX - halfWidth;
      double maxX = gridCenterX + halfWidth;
      double minY = gridCenterY - halfWidth;
      double maxY = gridCenterY + halfWidth;
      boundingBox.getMinPoint().set(minX, minY);
      boundingBox.getMaxPoint().set(maxX, maxY);

      centroids.clear();
      normals.clear();
      for (int i = 0; i < cellsPerSide * cellsPerSide; i++)
      {
         centroids.add();
         normals.add();
      }

      reset();
   }

   public void reset()
   {
      occupiedCells.reset();
      varianceDataMap.reset();
      countDataMap.reset();

      for (Point3DBasics centroid : centroids)
         centroid.setToNaN();
      for (Vector3DBasics normal : normals)
         normal.setToNaN();
      varianceDataMap.fill(0, cellsPerSide * cellsPerSide, Double.NaN);
      countDataMap.fill(0, cellsPerSide * cellsPerSide, -1);

      occupiedBoundingBox.setToNaN();
   }

   public double getCellX(int element)
   {
      return HeightMapTools.keyToXCoordinate(element, gridCenter.getX(), gridResolution, centerIndex);
   }

   public double getCellY(int element)
   {
      return HeightMapTools.keyToXCoordinate(element, gridCenter.getY(), gridResolution, centerIndex);
   }

   public double getCellCentroidX(int element)
   {
      return centroids.get(element).getX();
   }

   public double getCellCentroidY(int element)
   {
      return centroids.get(element).getY();
   }

   public double getCellZ(int element)
   {
      return centroids.get(element).getZ();
   }

   public double getVariance(int element)
   {
      return varianceDataMap.get(element);
   }

   public Point2DReadOnly getGridCenter()
   {
      return gridCenter;
   }

   public double getGridResolution()
   {
      return gridResolution;
   }

   public int getCellsPerSide()
   {
      return cellsPerSide;
   }

   public int getXIndex(double xPosition)
   {
      return HeightMapTools.coordinateToIndex(xPosition, gridCenter.getX(), gridResolution, centerIndex);
   }

   public int getYIndex(double yPosition)
   {
      return HeightMapTools.coordinateToIndex(yPosition, gridCenter.getY(), gridResolution, centerIndex);
   }

   public double getHeightAtPoint(Point2DReadOnly point)
   {
      return getHeightAtPoint(point.getX(), point.getY());
   }

   public double getHeightAtPoint(double x, double y)
   {
      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), gridResolution, centerIndex);
      return centroids.get(key).getZ();
   }

   public double getVarianceAtPoint(Point2DReadOnly point)
   {
      return getVarianceAtPoint(point.getX(), point.getY());
   }

   public double getVarianceAtPoint(double x, double y)
   {
      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), gridResolution, centerIndex);
      return varianceDataMap.get(key);
   }

   public double getNumberOfPointsAtPoint(Point2DReadOnly point)
   {
      return getNumberOfPointsAtPoint(point.getX(), point.getY());
   }

   public double getNumberOfPointsAtPoint(double x, double y)
   {
      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), gridResolution, centerIndex);
      return countDataMap.get(key);
   }

   public BoundingBox2D getOccupiedBoundingBox()
   {
      return occupiedBoundingBox;
   }

   public void updateFromFloatBufferImage(Mat centroidXBuffer,
                                          Mat centroidYBuffer,
                                          Mat centroidZBuffer,
                                          Mat varianceZBuffer,
                                          Mat normalXBuffer,
                                          Mat normalYBuffer,
                                          Mat normalZBuffer,
                                          Mat countMat)
   {
      occupiedBoundingBox.setToNaN();
      for (int y = 0; y < cellsPerSide; y++)
      {
         for (int x = 0; x < cellsPerSide; x++)
         {
            double xPosition = centroidXBuffer.ptr(y, x).getFloat();
            double yPosition = centroidYBuffer.ptr(y, x).getFloat();

            int count = Byte.toUnsignedInt(countMat.ptr(y, x).get());

            if (count > 0)
            {
               int key = HeightMapTools.coordinateToKey(xPosition, yPosition, gridCenter.getX(), gridCenter.getY(), gridResolution, centerIndex);

               if (key >= 0 && centroids.size() > key) // TODO: Fix this bug
               {
                  double nx = normalXBuffer.ptr(y, x).getFloat();
                  double ny = normalYBuffer.ptr(y, x).getFloat();
                  double nz = normalZBuffer.ptr(y, x).getFloat();

                  occupiedCells.add(key);
                  centroids.get(key).set(xPosition, yPosition, centroidZBuffer.ptr(y, x).getFloat());
                  normals.get(key).set(nx, ny, nz);
                  varianceDataMap.set(key, varianceZBuffer.ptr(y, x).getFloat());

                  occupiedBoundingBox.updateToIncludePoint(xPosition, yPosition);
                  countDataMap.set(key, count);
               }
            }
         }
      }
   }

   public HeightMapMessage buildMessage()
   {
      // Copy and report over messager
      HeightMapMessage message = new HeightMapMessage();
      message.setGridSizeXy(gridSizeXY);
      message.setXyResolution(gridResolution);
      message.setGridCenterX(gridCenter.getX());
      message.setGridCenterY(gridCenter.getY());

      for (int i = 0; i < occupiedCells.size(); i++)
      {
         message.getKeys().add(occupiedCells.get(i));
         message.getHeights().add((float) centroids.get(occupiedCells.get(i)).getZ());
         message.getVariances().add((float) varianceDataMap.get(occupiedCells.get(i)));
         message.getCentroids().add().set(centroids.get(occupiedCells.get(i)));
         message.getNormals().add().set(normals.get(occupiedCells.get(i)));
      }

      return message;
   }

   public RecyclingArrayList<Point3DBasics> getCentroids()
   {
      return centroids;
   }
}

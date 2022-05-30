package us.ihmc.perception.gpuHeightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.heightMap.HeightMapTools;

public class SimpleGPUHeightMap
{
   private final Point2D gridCenter = new Point2D();
   private double gridResolution;
   private double gridSizeXY;
   private int cellsPerSide;
   private int centerIndex;

   private final TIntArrayList occupiedCells = new TIntArrayList();

   private final TDoubleArrayList xDataMap = new TDoubleArrayList();
   private final TDoubleArrayList yDataMap = new TDoubleArrayList();
   private final TDoubleArrayList zDataMap = new TDoubleArrayList();
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

      reset();
   }

   public void reset()
   {
      occupiedCells.reset();
      xDataMap.reset();
      yDataMap.reset();
      zDataMap.reset();
      varianceDataMap.reset();
      countDataMap.reset();

      xDataMap.fill(0, cellsPerSide * cellsPerSide, Double.NaN);
      yDataMap.fill(0, cellsPerSide * cellsPerSide, Double.NaN);
      zDataMap.fill(0, cellsPerSide * cellsPerSide, Double.NaN);
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
      return xDataMap.get(element);
   }

   public double getCellCentroidY(int element)
   {
      return yDataMap.get(element);
   }

   public double getCellZ(int element)
   {
      return zDataMap.get(element);
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
      return zDataMap.get(key);
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
                                          Mat countMat)
   {
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

               occupiedCells.add(key);
               xDataMap.set(key, xPosition);
               yDataMap.set(key, yPosition);
               zDataMap.set(key, centroidZBuffer.ptr(y, x).getFloat());
               varianceDataMap.set(key, varianceZBuffer.ptr(y, x).getFloat());

               boundingBox.updateToIncludePoint(xPosition, yPosition);
               countDataMap.set(key, count);
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
         message.getHeights().add((float) zDataMap.get(occupiedCells.get(i)));
      }


      return message;
   }
}

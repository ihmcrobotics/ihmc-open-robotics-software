package us.ihmc.sensorProcessing.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HeightMapManager
{
   public static final double defaultVariance = 0.1;
   private static final boolean debug = false;

   /*  From HeightMapMessage.msg  */
   public static final int maxCellCount = 30000;

   private double maxHeight = 0.4;
   private double gridSizeXY;
   private double gridResolutionXY;
   private int centerIndex;
   private int cellsPerAxis;
   private HeightMapCell[] heightMapCells;
   private TIntArrayList occupiedCells = new TIntArrayList();

   private final HeightMapParametersReadOnly parameters;
   private final Point2D gridCenterXY = new Point2D();

   public HeightMapManager(HeightMapParametersReadOnly parameters, double gridResolutionXY, double gridSizeXY)
   {
      this.parameters = parameters;
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = 2 * centerIndex + 1;

      this.heightMapCells = new HeightMapCell[cellsPerAxis * cellsPerAxis];

      clear();
   }

   /**
    * Updates the grid size of the underlying height map. If this grid size is different from the current, it will
    * clear the current height map and build a new one.
    */
   public void updateGridSizeXY(double gridSizeXY)
   {
      if (MathTools.epsilonEquals(gridSizeXY, this.gridSizeXY, 1e-5))
         return;

      this.gridSizeXY = gridSizeXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = 2 * centerIndex + 1;

      heightMapCells = new HeightMapCell[cellsPerAxis * cellsPerAxis];
      occupiedCells.reset();
   }

   /**
    * Updates the grid resolution of the underlying height map. If this grid resolution is different from the current, it will
    * clear the current height map and build a new one.
    */
   public void updateGridResolutionXY(double gridResolutionXY)
   {
      if (MathTools.epsilonEquals(gridResolutionXY, this.gridResolutionXY, 1e-5))
         return;

      this.gridResolutionXY = gridResolutionXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = 2 * centerIndex + 1;

      heightMapCells = new HeightMapCell[cellsPerAxis * cellsPerAxis];
      occupiedCells.reset();
   }

   /**
    * Clears height map data and moves grid center to the given value.
    */
   public void resetAtGridCenter(double xCenter, double yCenter)
   {
      gridCenterXY.set(xCenter, yCenter);
      clear();
   }

   /**
    * Translates the existing height map to a new center location. It keeps all the cells that are still in range and translates them to new locations.
    */
   public void translateToNewGridCenter(Point2DReadOnly gridCenter, double varianceToAdd)
   {
      translateToNewGridCenter(gridCenter.getX(), gridCenter.getY(), varianceToAdd);
   }

   /**
    * Translates the existing height map to a new center location. It keeps all the cells that are still in range and translates them to new locations.
    */
   public void translateToNewGridCenter(double xCenter, double yCenter, double varianceToAdd)
   {
//      int xIndexShift = HeightMapTools.coordinateToIndex(xCenter - this.gridCenterXY.getX(), 0.0, gridResolutionXY, centerIndex);
//      int yIndexShift = HeightMapTools.coordinateToIndex(yCenter - this.gridCenterXY.getY(), 0.0, gridResolutionXY, centerIndex);
//      if (xIndexShift == 0 && yIndexShift == 0)
//         return;

      if ((Math.abs(xCenter - this.gridCenterXY.getX()) < gridResolutionXY / 2.0) && (Math.abs(yCenter - this.gridCenterXY.getY()) < gridResolutionXY / 2.0))
         return;

      HeightMapCell[] oldCellArray = heightMapCells;
      TIntArrayList oldOccupiedCells = occupiedCells;
      heightMapCells = new HeightMapCell[cellsPerAxis * cellsPerAxis];
      occupiedCells = new TIntArrayList();

      for (int i = 0; i < oldOccupiedCells.size(); i++)
      {
         int oldKey = oldOccupiedCells.get(i);
         if (oldCellArray[oldKey] == null)
            continue;

         int oldXIndex = HeightMapTools.keyToXIndex(oldKey, centerIndex);
         int oldYIndex = HeightMapTools.keyToYIndex(oldKey, centerIndex);

         double xCoordinate = HeightMapTools.keyToXCoordinate(oldKey, gridCenterXY.getX(), gridResolutionXY, centerIndex);
         double yCoordinate = HeightMapTools.keyToYCoordinate(oldKey, gridCenterXY.getY(), gridResolutionXY, centerIndex);

         int xIndex = HeightMapTools.coordinateToIndex(xCoordinate, xCenter, gridResolutionXY, centerIndex);
//         int xIndex = oldXIndex + xIndexShift;
         if (xIndex < 0 || xIndex >= cellsPerAxis)
         {
            continue;
         }

         int yIndex = HeightMapTools.coordinateToIndex(yCoordinate, yCenter, gridResolutionXY, centerIndex);
//         int yIndex = oldYIndex + yIndexShift;
         if (yIndex < 0 || yIndex >= cellsPerAxis)
         {
            continue;
         }

         int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
         heightMapCells[key] = oldCellArray[oldKey];
         if (key != oldKey)
            heightMapCells[key].addVariance(varianceToAdd);
         occupiedCells.add(key);
      }

      gridCenterXY.set(xCenter, yCenter);
   }

   /**
    * Clears height map data
    */
   public void clear()
   {
      Arrays.fill(heightMapCells, null);
      occupiedCells.clear();
   }

   /**
    * This method consums the {@param pointCloud} add adds it to the existing height map. It then updates the height map estimates in each of the cells.
    */
   public void update(Point3D[] pointCloud)
   {
      update(pointCloud, 1.0);
   }

   /**
    * This method consums the {@param pointCloud} add adds it to the existing height map. It then updates the height map estimates in each of the cells.
    * It does this using a constant variance for all points, defined by {@param verticalMeasurementVariance}.
    */
   public void update(Point3D[] pointCloud, double verticalMeasurementVariance)
   {
//      List<Point3D> pointList = new ArrayList<>();
//
//      for (int i = -2; i <= 2; i++)
//      {
//         for (int j = -2; j <= 2; j++)
//         {
//            if (i == 0 && j == 0)
//               continue;
//            if (i == 0 && j == 1)
//               continue;
//            if (i == 1 && j == 0)
//               continue;
//
//            pointList.add(new Point3D(i * gridResolutionXY + gridCenterXY.getX(), j * gridResolutionXY + gridCenterXY.getY(), -0.3));
//         }
//      }

//      for (int i = -9; i < 9; i++)
//      {
//         for (int j = -9; j < 9; j++)
//         {
//            if (Math.abs(i) <= 2 && Math.abs(j) <= 2)
//               continue;
//
//            pointList.add(new Point3D(i * gridResolutionXY + gridCenterXY.getX(), j * gridResolutionXY + gridCenterXY.getY(), 0.0));
//         }
//      }

//      Point3D[] pointCloud = pointList.toArray(new Point3D[0]);

      for (int i = 0; i < pointCloud.length; i++)
      {
         if (pointCloud[i] != null)
         {
            Point3DReadOnly point = pointCloud[i];

            // cinders
            //            if (point.getZ() > 0.4)
            //               continue;

            // stairs side
            //            if (point.getX() < 0.5 && point.getZ() > 0.3)
            //               continue;
            //            if (point.getY() > 1.0 && point.getZ() > 0.3)
            //               continue;

            // stairs
            //            if (point.getX() < 0.6 && point.getZ() > -0.2)
            //               continue;

            // narrow passage
            //            if ((point.getX() < 0.6 || point.getY() < -1.5) && point.getZ() > 0.2)
            //               continue;

            // stepping stones
            //            if (point.getZ() > 0.4)
            //               continue;

            if (point.getZ() > maxHeight)
            {
               continue;
            }

            int xIndex = HeightMapTools.coordinateToIndex(point.getX(), gridCenterXY.getX(), gridResolutionXY, centerIndex);
            if (xIndex < 0 || xIndex >= cellsPerAxis)
               continue;

            int yIndex = HeightMapTools.coordinateToIndex(point.getY(), gridCenterXY.getY(), gridResolutionXY, centerIndex);
            if (yIndex < 0 || yIndex >= cellsPerAxis)
               continue;

            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            boolean noCellPresent = heightMapCells[key] == null;

            if (noCellPresent && occupiedCells.size() >= maxCellCount)
            {
               continue;
            }

            if (noCellPresent)
            {
               heightMapCells[key] = new HeightMapCell(parameters);
               occupiedCells.add(key);
            }

            heightMapCells[key].addPoint(point.getZ(), verticalMeasurementVariance);
         }
      }

      for (int i = 0; i < occupiedCells.size(); i++)
         heightMapCells[occupiedCells.get(i)].updateHeightEstimate();

      if (debug)
         LogTools.info(occupiedCells.size() + " cells");
   }

   public double getHeightAt(double x, double y)
   {
      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenterXY.getX(), gridResolutionXY, centerIndex);
      if (xIndex < 0 || xIndex >= cellsPerAxis)
         return Double.NaN;

      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenterXY.getY(), gridResolutionXY, centerIndex);
      if (yIndex < 0 || yIndex >= cellsPerAxis)
         return Double.NaN;

      return getHeightAt(xIndex, yIndex);
   }

   public double getHeightAt(int indexX, int indexY)
   {
      return heightMapCells[HeightMapTools.indicesToKey(indexX, indexY, centerIndex)].getEstimatedHeight();
   }

   public TIntArrayList getOccupiedCells()
   {
      return occupiedCells;
   }

   public int getNumberOfCells()
   {
      return occupiedCells.size();
   }

   public int getXIndex(int i)
   {
      return HeightMapTools.keyToXIndex(occupiedCells.get(i), centerIndex);
   }

   public int getYIndex(int i)
   {
      return HeightMapTools.keyToYIndex(occupiedCells.get(i), centerIndex);
   }

   public double getHeightAt(int i)
   {
      return heightMapCells[occupiedCells.get(i)].getEstimatedHeight();
   }

   public int getKey(int i)
   {
      return occupiedCells.get(i);
   }

   public void setGroundCell(int i, boolean isGroundCell)
   {
      heightMapCells[occupiedCells.get(i)].setGroundCell(isGroundCell);
   }

   public void setHasSufficientNeighbors(int i, boolean hasSufficientNeighbors)
   {
      heightMapCells[occupiedCells.get(i)].setHasSufficientNeighbors(hasSufficientNeighbors);
   }

   public void resetAtHeight(int i, double height, double variance)
   {
      heightMapCells[occupiedCells.get(i)].resetAtHeight(height, variance);
   }

   public void resetAtHeightByKey(int key, double height, double variance)
   {
      if (heightMapCells[key] == null)
      {
         heightMapCells[key] = new HeightMapCell(parameters);
         occupiedCells.add(key);
      }
      else
      {
         throw new RuntimeException("Should not get here");
      }

      heightMapCells[key].resetAtHeight(height, variance);
   }

   public boolean cellHasData(int key)
   {
      return occupiedCells.contains(key);
   }

   public boolean cellHasData(int xIndex, int yIndex)
   {
      return occupiedCells.contains(HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex));
   }

   public boolean cellHasUnfilteredData(int i)
   {
      HeightMapCell cell = heightMapCells[occupiedCells.get(i)];
      return !cell.isGroundCell() && cell.hasSufficientNeighbors();
   }

   public boolean cellHasUnfilteredData(int xIndex, int yIndex)
   {
      int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
      HeightMapCell cell = heightMapCells[key];
      if (cell == null)
         return false;
      return !cell.isGroundCell() && cell.hasSufficientNeighbors();
   }

   public boolean isGroundCell(int i)
   {
      return heightMapCells[occupiedCells.get(i)].isGroundCell();
   }

   public boolean hasSufficientNeighbors(int i)
   {
      return heightMapCells[occupiedCells.get(i)].hasSufficientNeighbors();
   }

   public boolean hasSufficientNeighbors(int xIndex, int yIndex)
   {
      return heightMapCells[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)].hasSufficientNeighbors();
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public Point2DReadOnly getGridCenterXY()
   {
      return gridCenterXY;
   }

   public void setMaxHeight(double maxHeight)
   {
      this.maxHeight = maxHeight;
   }
}

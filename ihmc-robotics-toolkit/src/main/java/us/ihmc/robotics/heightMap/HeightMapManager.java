package us.ihmc.robotics.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

import java.util.Arrays;

public class HeightMapManager
{
   private static final boolean debug = false;

   /*  From HeightMapMessage.msg  */
   public static final int maxCellCount = 30000;

   private double maxHeight = 0.4;
   private final double gridResolutionXY;
   private final int centerIndex;
   private final int cellsPerAxis;
   private final HeightMapCell[] heightMapCells;
   private final TIntArrayList occupiedCells = new TIntArrayList();

   private final HeightMapParametersReadOnly parameters;
   private final Point2D gridCenterXY = new Point2D();

   public HeightMapManager(HeightMapParametersReadOnly parameters, double gridResolutionXY, double gridSizeXY)
   {
      this.parameters = parameters;
      this.gridResolutionXY = gridResolutionXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = 2 * centerIndex + 1;

      this.heightMapCells = new HeightMapCell[cellsPerAxis * cellsPerAxis];

      clear();
   }

   /**
    * Clears height map data and moves grid center to the given value.
    */
   public void setGridCenter(double xCenter, double yCenter)
   {
      gridCenterXY.set(xCenter, yCenter);
      clear();
   }

   /**
    * Clears height map data
    */
   public void clear()
   {
      Arrays.fill(heightMapCells, null);
      occupiedCells.clear();
   }

   public void update(Point3D[] pointCloud)
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
            Point3D point = new Point3D(pointCloud[i]);

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

            heightMapCells[key].addPoint(point.getZ());
         }
      }

      if (debug)
         LogTools.info(occupiedCells.size() + " cells");
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

   public void resetAtHeight(int i, double height)
   {
      heightMapCells[occupiedCells.get(i)].resetAtHeight(height);
   }

   public void resetAtHeightByKey(int key, double height)
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

      heightMapCells[key].resetAtHeight(height);
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

   public Point2D getGridCenterXY()
   {
      return gridCenterXY;
   }

   public void setMaxHeight(double maxHeight)
   {
      this.maxHeight = maxHeight;
   }
}

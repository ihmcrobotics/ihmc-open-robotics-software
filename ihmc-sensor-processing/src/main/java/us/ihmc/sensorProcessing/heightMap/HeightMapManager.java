package us.ihmc.sensorProcessing.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

import java.util.Arrays;

public class HeightMapManager
{
   private static final boolean debug = false;

   /*  From HeightMapMessage.msg  */
   private static final int maxCellCount = 30000;

   private final double gridResolutionXY;
   private final int minMaxIndexXY;
   private final int cellsPerAxis;
   private final HeightMapCell[] heightMapCells;
   private final TIntArrayList occupiedCells = new TIntArrayList();

   private final HeightMapParameters parameters = new HeightMapParameters(null);
   private final Point2D gridCenterXY = new Point2D();

   public HeightMapManager(double gridResolutionXY, double gridSizeXY)
   {
      this.gridResolutionXY = gridResolutionXY;
      minMaxIndexXY = HeightMapTools.minMaxIndex(gridSizeXY, gridResolutionXY);

      cellsPerAxis = 2 * minMaxIndexXY + 1;
      heightMapCells = new HeightMapCell[cellsPerAxis * cellsPerAxis];
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
            if (point.getZ() > 0.4)
               continue;

            int indexX = HeightMapTools.toIndex(point.getX(), gridCenterXY.getX(), gridResolutionXY, minMaxIndexXY);
            int indexY = HeightMapTools.toIndex(point.getY(), gridCenterXY.getY(), gridResolutionXY, minMaxIndexXY);
            int indexXY = HeightMapTools.toXYIndex(indexX, indexY, minMaxIndexXY);

            if (indexX < 0 || indexY < 0 || indexX >= cellsPerAxis || indexY >= cellsPerAxis)
            {
               continue;
            }

            boolean noCellPresent = heightMapCells[indexXY] == null;
            if (noCellPresent && occupiedCells.size() >= maxCellCount)
            {
               continue;
            }

            if (noCellPresent)
            {
               heightMapCells[indexXY] = new HeightMapCell(parameters);
               occupiedCells.add(indexXY);
            }

            heightMapCells[indexXY].addPoint(point.getZ());
         }
      }

      if (debug)
         LogTools.info(occupiedCells.size() + " cells");
   }

   public double getHeightAt(int indexX, int indexY)
   {
      return heightMapCells[HeightMapTools.toXYIndex(indexX, indexY, minMaxIndexXY)].getEstimatedHeight();
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
      return HeightMapTools.xIndex(occupiedCells.get(i), minMaxIndexXY);
   }

   public int getYIndex(int i)
   {
      return HeightMapTools.yIndex(occupiedCells.get(i), minMaxIndexXY);
   }

   public double getHeightAt(int i)
   {
      return heightMapCells[occupiedCells.get(i)].getEstimatedHeight();
   }

   public void clearCell(int i)
   {
      heightMapCells[occupiedCells.get(i)] = null;
      occupiedCells.removeAt(i);
   }

   public void resetAtHeight(int i, double height)
   {
      heightMapCells[occupiedCells.get(i)].resetAtHeight(height);
   }

   public boolean isCellPresent(int xIndex, int yIndex)
   {
      return occupiedCells.contains(HeightMapTools.toXYIndex(xIndex, yIndex, minMaxIndexXY));
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public Point2D getGridCenterXY()
   {
      return gridCenterXY;
   }
}

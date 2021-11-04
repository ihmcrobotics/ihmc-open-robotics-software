package us.ihmc.sensorProcessing.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

public class HeightMapManager
{
   private static final boolean debug = false;

   /*  From HeightMapMessage.msg  */
   private static final int maxCellCount = 30000;

   private final double gridResolutionXY;
   private final int minMaxIndexXY;
   private final int cellsPerAxis;
   private HeightMapCell[][] heightMapCells;

   private final HeightMapParameters parameters = new HeightMapParameters(null);
   private final Point2D gridCenterXY = new Point2D();

   private final TIntArrayList xCells = new TIntArrayList();
   private final TIntArrayList yCells = new TIntArrayList();

   public HeightMapManager(double gridResolutionXY, double gridSizeXY)
   {
      this.gridResolutionXY = gridResolutionXY;
      minMaxIndexXY = HeightMapTools.minMaxIndex(gridSizeXY, gridResolutionXY);

      cellsPerAxis = 2 * minMaxIndexXY + 1;
      heightMapCells = new HeightMapCell[cellsPerAxis][cellsPerAxis];
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
      heightMapCells = new HeightMapCell[cellsPerAxis][cellsPerAxis];
      xCells.clear();
      yCells.clear();
   }

   public void update(Point3D[] pointCloud)
   {
      for (int i = 0; i < pointCloud.length; i++)
      {
         if (pointCloud[i] != null)
         {
            Point3D point = new Point3D(pointCloud[i]);

            int indexX = HeightMapTools.toIndex(point.getX(), gridCenterXY.getX(), gridResolutionXY, minMaxIndexXY);
            int indexY = HeightMapTools.toIndex(point.getY(), gridCenterXY.getY(), gridResolutionXY, minMaxIndexXY);

            if (indexX < 0 || indexY < 0 || indexX >= heightMapCells.length || indexY >= heightMapCells.length)
            {
               continue;
            }

            boolean noCellPresent = heightMapCells[indexX][indexY] == null;
            if (noCellPresent && xCells.size() >= maxCellCount)
            {
               continue;
            }

            if (noCellPresent)
            {
               heightMapCells[indexX][indexY] = new HeightMapCell(parameters);
               xCells.add(indexX);
               yCells.add(indexY);
            }

            heightMapCells[indexX][indexY].addPoint(point.getZ());
         }
      }

      if (debug)
         LogTools.info(xCells.size() + " cells");
   }

   public double getHeightAt(int indexX, int indexY)
   {
      return heightMapCells[indexX][indexY].getEstimatedHeight();
   }

   public TIntArrayList getXCells()
   {
      return xCells;
   }

   public TIntArrayList getYCells()
   {
      return yCells;
   }

   public Point2D getGridCenterXY()
   {
      return gridCenterXY;
   }
}

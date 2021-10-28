package us.ihmc.sensorProcessing.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

public class HeightMapManager
{
   private static final boolean debug = false;

   private static final double MIN_X = -2.0;
   private static final double MAX_X = 2.0;
   private static final double MIN_Y = -2.0;
   private static final double MAX_Y = 2.0;
   private static final double MIN_Z = -10.0;
   private static final double MAX_Z = 1.0;

   private final double gridResolutionXY;
   private final int minMaxIndexXY;
   private final HeightMapCell[][] heightMapCells;

   private final HeightMapParameters parameters = new HeightMapParameters(null);
   private final Point2D gridCenterXY = new Point2D();

   private final TIntArrayList xCells = new TIntArrayList();
   private final TIntArrayList yCells = new TIntArrayList();

   public HeightMapManager(double gridResolutionXY, double gridSizeXY)
   {
      this.gridResolutionXY = gridResolutionXY;
      minMaxIndexXY = HeightMapTools.toIndex(gridSizeXY, gridResolutionXY, 0);

      int cellsPerAxis = 2 * minMaxIndexXY + 1;
      heightMapCells = new HeightMapCell[cellsPerAxis][cellsPerAxis];
   }

   public void initialize(double xCenter, double yCenter)
   {
      gridCenterXY.set(xCenter, yCenter);
   }

   public void update(Point3D[] pointCloud)
   {
      for (int i = 0; i < pointCloud.length; i++)
      {
         if (pointCloud[i] != null)
         {
            Point3D point = new Point3D(pointCloud[i]);
            if (!MathTools.intervalContains(point.getX(), MIN_X, MAX_X) ||
                !MathTools.intervalContains(point.getY(), MIN_Y, MAX_Y) ||
                !MathTools.intervalContains(point.getZ(), MIN_Z, MAX_Z))
               continue;

            int indexX = HeightMapTools.toIndex(point.getX() - gridCenterXY.getX(), gridResolutionXY, minMaxIndexXY);
            int indexY = HeightMapTools.toIndex(point.getY() - gridCenterXY.getY(), gridResolutionXY, minMaxIndexXY);
            if (indexX < 0 || indexY < 0 || indexX >= heightMapCells.length || indexY >= heightMapCells.length)
            {
               continue;
            }

             if (heightMapCells[indexX][indexY] == null)
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
}

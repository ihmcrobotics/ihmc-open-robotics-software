package us.ihmc.sensorProcessing.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;

import java.util.Arrays;

public class HeightMapData
{
   private final TIntArrayList occupiedCells = new TIntArrayList();
   private final double[] heights;

   private final int minMaxIndexXY;
   private final int cellsPerAxis;
   private final double gridResolutionXY;
   private final double gridSizeXY;
   private final Point2D gridCenter = new Point2D();
   private double estimatedGroundHeight = Double.NaN;

   public HeightMapData(double gridResolutionXY, double gridSizeXY, double gridCenterX, double gridCenterY)
   {
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      this.minMaxIndexXY = HeightMapTools.minMaxIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = (2 * minMaxIndexXY + 1);
      this.heights = new double[cellsPerAxis * cellsPerAxis];
      this.gridCenter.set(gridCenterX, gridCenterY);

      reset();
   }

   public HeightMapData(HeightMapMessage heightMapMessage)
   {
      this(heightMapMessage.getXyResolution(), heightMapMessage.getGridSizeXy(), heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY());

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         double height = heightMapMessage.getHeights().get(i);
         int xyIndex = heightMapMessage.getCells().get(i);
         heights[xyIndex] = height;
         occupiedCells.add(xyIndex);
      }

      this.estimatedGroundHeight = heightMapMessage.getEstimatedGroundHeight();
   }

   public void reset()
   {
      occupiedCells.clear();
      Arrays.fill(heights, Double.NaN);
      estimatedGroundHeight = Double.NaN;
   }

   public double getGridResolutionXY()
   {
      return gridResolutionXY;
   }

   public double getGridSizeXY()
   {
      return gridSizeXY;
   }

   public int getNumberOfCells()
   {
      return occupiedCells.size();
   }

   public double getHeight(int i)
   {
      return heights[occupiedCells.get(i)];
   }

   public Point2D getCellPosition(int i)
   {
      int cell = occupiedCells.get(i);
      int indexX = HeightMapTools.xIndex(cell, minMaxIndexXY);
      int indexY = HeightMapTools.yIndex(cell, minMaxIndexXY);
      return new Point2D(HeightMapTools.toCoordinate(indexX, gridCenter.getX(), gridResolutionXY, minMaxIndexXY),
                         HeightMapTools.toCoordinate(indexY, gridCenter.getY(), gridResolutionXY, minMaxIndexXY));
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeightAt(double x, double y)
   {
      int xIndex = HeightMapTools.toIndex(x, gridCenter.getX(), gridResolutionXY, minMaxIndexXY);
      int yIndex = HeightMapTools.toIndex(y, gridCenter.getY(), gridResolutionXY, minMaxIndexXY);

      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         LogTools.error("Invalid index for point (" + x + ", " + y + "). Indices: (" + xIndex + ", " + yIndex + ")");
         return Double.NaN;
      }

      int xyIndex = HeightMapTools.toXYIndex(xIndex, yIndex, minMaxIndexXY);
      if (occupiedCells.contains(xyIndex))
      {
         return heights[xyIndex];
      }
      else
      {
         return estimatedGroundHeight;
      }
   }

   public void setHeightAt(double x, double y, double z)
   {
      int xIndex = HeightMapTools.toIndex(x, gridCenter.getX(), gridResolutionXY, minMaxIndexXY);
      int yIndex = HeightMapTools.toIndex(y, gridCenter.getY(), gridResolutionXY, minMaxIndexXY);

      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         LogTools.error("Invalid index for point (" + x + ", " + y + "). Indices: (" + xIndex + ", " + yIndex + ")");
         return;
      }

      int xyIndex = HeightMapTools.toXYIndex(xIndex, yIndex, minMaxIndexXY);
      if (Double.isNaN(heights[xyIndex]))
      {
         occupiedCells.add(xyIndex);
      }

      heights[xyIndex] = z;
   }

   public double getHeightAt(int xIndex, int yIndex)
   {
      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         LogTools.error("Invalid index (" + xIndex + ", " + yIndex + ")");
         return Double.NaN;
      }

      return heights[HeightMapTools.toXYIndex(xIndex, yIndex, minMaxIndexXY)];
   }

   public void setEstimatedGroundHeight(double estimatedGroundHeight)
   {
      this.estimatedGroundHeight = estimatedGroundHeight;
   }

   public double getEstimatedGroundHeight()
   {
      return estimatedGroundHeight;
   }

   public int getMinMaxIndexXY()
   {
      return minMaxIndexXY;
   }

   public Point2D getGridCenter()
   {
      return gridCenter;
   }

   public double minHeight()
   {
      double minValue = Double.POSITIVE_INFINITY;
      for (int i = 0; i < heights.length; i++)
      {
         if (!Double.isNaN(heights[i]) && heights[i] < minValue)
            minValue = heights[i];
      }

      return minValue;
   }
}

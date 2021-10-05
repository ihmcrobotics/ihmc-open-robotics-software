package us.ihmc.sensorProcessing.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.log.LogTools;

import java.util.Arrays;

public class HeightMapData
{
   private final TIntArrayList xCells = new TIntArrayList();
   private final TIntArrayList yCells = new TIntArrayList();
   private final TDoubleArrayList heights = new TDoubleArrayList();
   private double[][] sortedHeights;

   private final int minMaxIndexXY;
   private final double gridResolutionXY;
   private final double gridSizeXY;

   public HeightMapData(double gridResolutionXY, double gridSizeXY)
   {
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      minMaxIndexXY = HeightMapTools.toIndex(gridSizeXY, gridResolutionXY, 0);
   }

   public HeightMapData(HeightMapMessage heightMapMessage)
   {
      this.gridResolutionXY = heightMapMessage.getXyResolution();
      this.gridSizeXY = heightMapMessage.getGridSizeXy();
      minMaxIndexXY = HeightMapTools.toIndex(gridSizeXY, gridResolutionXY, 0);

      xCells.addAll(heightMapMessage.getXCells());
      yCells.addAll(heightMapMessage.getYCells());
      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         heights.add(heightMapMessage.getHeights().get(i));
      }

      sort();
   }

   public TIntArrayList getXCells()
   {
      return xCells;
   }

   public TIntArrayList getYCells()
   {
      return yCells;
   }

   public TDoubleArrayList getHeights()
   {
      return heights;
   }

   public double getGridResolutionXY()
   {
      return gridResolutionXY;
   }

   public double getGridSizeXY()
   {
      return gridSizeXY;
   }

   public void sort()
   {
      int cellsPerAxis = 2 * minMaxIndexXY + 1;
      sortedHeights = new double[cellsPerAxis][cellsPerAxis];

      // initialize with NaN
      for (int i = 0; i < sortedHeights.length; i++)
      {
         Arrays.fill(sortedHeights[i], Double.NaN);
      }

      // set height data
      for (int i = 0; i < xCells.size(); i++)
      {
         sortedHeights[xCells.get(i)][yCells.get(i)] = heights.get(i);
      }
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeightAt(double x, double y)
   {
      if (sortedHeights == null)
      {
         LogTools.error("Sorted heights is null. Must call sort() before this is available");
         return Double.NaN;
      }

      int xIndex = HeightMapTools.toIndex(x, gridResolutionXY, minMaxIndexXY);
      int yIndex = HeightMapTools.toIndex(y, gridResolutionXY, minMaxIndexXY);

      if (xIndex < 0 || yIndex < 0 || xIndex >= sortedHeights.length || yIndex >= sortedHeights.length)
      {
         LogTools.error("Invalid index for point (" + x + ", " + y + "). Indices: (" + xIndex + ", " + yIndex + ")");
      }

      return sortedHeights[xIndex][yIndex];
   }

   public double getHeightAt(int xIndex, int yIndex)
   {
      return sortedHeights[xIndex][yIndex];
   }

   public void clear()
   {
      xCells.clear();
      yCells.clear();
      heights.clear();
      sortedHeights = null;
   }
}

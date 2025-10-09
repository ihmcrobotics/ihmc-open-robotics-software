package us.ihmc.perception.gpuMapping;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;

public class HeightMapData
{
   /* List of heights indexed by key. See HeightMapTools for definition of key */
   private float[] heights;
   /* Zero-indexed key corresponding to the center of the height map */
   private int centerIndex;
   /* Number of cells along each axis of the height map */
   private int cellsPerAxis;
   /* Discretization size in meters of the height map, aka the map resolution */
   private double cellSize;
   /* Length of the height map along x/y in meters */
   private double mapSize;
   /* World-frame coordinate of the center of the height map */
   private final Point2D gridCenter = new Point2D();
   /* Convenience fields for the height map dimensions */
   private double minX, maxX, minY, maxY;

   public HeightMapData(double cellSize, double mapSize, double gridCenterX, double gridCenterY)
   {
      this.cellSize = cellSize;
      this.mapSize = mapSize;
      this.centerIndex = HeightMapTools.computeCenterIndex(mapSize, cellSize);
      this.cellsPerAxis = 2 * centerIndex + 1;
      this.heights = new float[cellsPerAxis * cellsPerAxis];
      this.gridCenter.set(gridCenterX, gridCenterY);
      updateGridDimensions();
   }

   public HeightMapData(HeightMapData latestHeightMapData)
   {
      set(latestHeightMapData);
   }

   public void set(HeightMapData latestHeightMapData)
   {
      this.cellSize = latestHeightMapData.getCellSize();
      this.mapSize = latestHeightMapData.getMapSize();
      this.centerIndex = HeightMapTools.computeCenterIndex(latestHeightMapData.getMapSize(), latestHeightMapData.getCellSize());
      this.cellsPerAxis = 2 * latestHeightMapData.getCenterIndex() + 1;
      this.heights = new float[latestHeightMapData.getCellsPerAxis() * latestHeightMapData.getCellsPerAxis()];

      for (int i = 0; i < latestHeightMapData.getCellsPerAxis() * latestHeightMapData.getCellsPerAxis(); i++)
      {
         this.heights[i] = latestHeightMapData.heights[i];
      }

      this.gridCenter.set(latestHeightMapData.getGridCenter());

      updateGridDimensions();
   }

   private void updateGridDimensions()
   {
      double epsilon = 1e-8;
      double halfWidth = 0.5 * (mapSize + cellSize) - epsilon;
      minX = gridCenter.getX() - halfWidth;
      maxX = gridCenter.getX() + halfWidth;
      minY = gridCenter.getY() - halfWidth;
      maxY = gridCenter.getY() + halfWidth;
   }

   public double getCellSize()
   {
      return cellSize;
   }

   public double getMapSize()
   {
      return mapSize;
   }

   public Point2D getCellPosition(int key)
   {
      return new Point2D(HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), cellSize, centerIndex),
                         HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), cellSize, centerIndex));
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeight(double x, double y)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         //LogTools.debug(String.format("Outside height map bounds: %.2f, %.2f, %.2f, %.2f", minX, maxX, minY, maxY));
         return Double.NaN;
      }

      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), cellSize, centerIndex);
      return heights[key];
   }

   public void setHeight(int key, double height)
   {
      if (key >= 0 && key < heights.length)
      {
         heights[key] = (float) height;
      }
   }

   public void setHeights(float[] heights)
   {
      this.heights = heights;
   }

   public void setHeight(double x, double y, double z)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         return;
      }

      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), cellSize, centerIndex);
      heights[key] = (float) z;
   }

   public double getHeight(int key)
   {
      return heights[key];
   }

   public double getHeight(int xIndex, int yIndex)
   {
      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         return Double.NaN;
      }

      return heights[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)];
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public Point2D getGridCenter()
   {
      return gridCenter;
   }

   public double getMinHeight()
   {
      double minValue = Double.POSITIVE_INFINITY;
      for (int i = 0; i < heights.length; i++)
      {
         if (!Double.isNaN(heights[i]) && heights[i] < minValue)
            minValue = heights[i];
      }

      return minValue;
   }

   public void setGridCenter(double x, double y)
   {
      gridCenter.set(x, y);
      updateGridDimensions();
   }

   public float[] getHeights()
   {
      return heights;
   }
}

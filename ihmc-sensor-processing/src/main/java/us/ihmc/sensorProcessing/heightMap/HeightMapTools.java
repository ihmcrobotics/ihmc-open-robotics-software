package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.log.LogTools;

/**
 * Height map indexing tools. The height map spans a square region and is parametrized by the following values:
 * - A discretization value
 * - The grid size, i.e. side length of the square region it covers
 * - Grid center, an xy coordinate which is the middle of the grid
 *
 * Cells are indexed two ways:
 * - A unique integer key, which is zero-indexed and starts at the corner of the grid which is the negative-most x and y coordinates.
 * - An (x,y) integer index pair, which is zero at the negative-most cell along each axis
 */
public class HeightMapTools
{
   /**
    * The xy-indices of the center of the grid.
    */
   public static int computeCenterIndex(double gridSize, double resolution)
   {
      return (int) Math.round(0.5 * gridSize / resolution);
   }

   public static int coordinateToKey(double x, double y, double xCenter, double yCenter, double resolution, int centerIndex)
   {
      int xIndex = coordinateToIndex(x, xCenter, resolution, centerIndex);
      int yIndex = coordinateToIndex(y, yCenter, resolution, centerIndex);
      return indicesToKey(xIndex, yIndex, centerIndex);
   }

   public static double keyToXCoordinate(int key, double xCenter, double resolution, int centerIndex)
   {
      int xIndex = keyToXIndex(key, centerIndex);
      return indexToCoordinate(xIndex, xCenter, resolution, centerIndex);
   }

   public static double keyToYCoordinate(int key, double yCenter, double resolution, int centerIndex)
   {
      int yIndex = keyToYIndex(key, centerIndex);
      return indexToCoordinate(yIndex, yCenter, resolution, centerIndex);
   }

   public static int coordinateToIndex(double coordinate, double gridCenter, double resolution, int centerIndex)
   {
      return (int) Math.round((coordinate - gridCenter) / resolution) + centerIndex;
   }

   public static double indexToCoordinate(int index, double gridCenter, double resolution, int centerIndex)
   {
      return (index - centerIndex) * resolution + gridCenter;
   }

   public static int keyToXIndex(int key, int centerIndex)
   {
      return key % (2 * centerIndex + 1);
   }

   public static int keyToYIndex(int key, int centerIndex)
   {
      return key / (2 * centerIndex + 1);
   }

   public static int indicesToKey(int xIndex, int yIndex, int centerIndex)
   {
      return xIndex + yIndex * (2 * centerIndex + 1);
   }

   public static int getIndexFromCoordinates(double coordinate, float resolution, int offset)
   {
      return (int) (coordinate * resolution + offset);
   }

   public static double getCoordinateFromIndex(int index, double resolution, int offset)
   {
      return (index - offset) / resolution;
   }
}

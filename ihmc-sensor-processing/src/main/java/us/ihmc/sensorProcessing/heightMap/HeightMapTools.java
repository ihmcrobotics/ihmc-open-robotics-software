package us.ihmc.sensorProcessing.heightMap;

/**
 * Tools for indexing height map cells.
 * The height map grid is centered at (0, 0), the grid resolution (cell width) is r and the grid is a square of width w, such that w/2 is the maxXYCoordinate
 */
public class HeightMapTools
{
   public static int toIndex(double coordinate, double gridCenter, double resolution, int minMaxIndexXY)
   {
      return (int) Math.round((coordinate - gridCenter) / resolution) + minMaxIndexXY;
   }

   public static double toCoordinate(int index, double gridCenter, double resolution, int minMaxIndexXY)
   {
      return (index - minMaxIndexXY) * resolution + gridCenter;
   }

   public static int minMaxIndex(double gridSizeXY, double resolution)
   {
      return (int) Math.round(0.5 * gridSizeXY / resolution);
   }

   /* Maps xy indices to single value */
   public static int toXYIndex(int xIndex, int yIndex, int minMaxIndexXY)
   {
      return xIndex + (2 * minMaxIndexXY + 1) * yIndex;
   }

   public static int xIndex(int xyIndex, int minMaxIndexXY)
   {
      return xyIndex % (2 * minMaxIndexXY + 1);
   }

   public static  int yIndex(int xyIndex, int minMaxIndexXY)
   {
      return xyIndex / (2 * minMaxIndexXY + 1);
   }

}

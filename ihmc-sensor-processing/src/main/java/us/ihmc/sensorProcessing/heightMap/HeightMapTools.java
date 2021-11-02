package us.ihmc.sensorProcessing.heightMap;

/**
 * Tools for indexing height map cells.
 * The height map grid is centered at (0, 0), the grid resolution (cell width) is r and the grid is a square of width w, such that w/2 is the maxXYCoordinate
 */
public class HeightMapTools
{
   public static int toIndex(double coordinate, double resolution, int minMaxIndexXY)
   {
      return (int) Math.round(coordinate / resolution) + minMaxIndexXY;
   }

   public static double toCoordinate(int index, double resolution, int minMaxIndexXY)
   {
      return (index - minMaxIndexXY) * resolution;
   }

   public static int minMaxIndex(double maxXYCoordinate, double resolution)
   {
      return (int) Math.round(maxXYCoordinate / resolution);
   }
}

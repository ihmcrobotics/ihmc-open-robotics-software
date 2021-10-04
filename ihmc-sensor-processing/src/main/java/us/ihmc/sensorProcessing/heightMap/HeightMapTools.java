package us.ihmc.sensorProcessing.heightMap;

public class HeightMapTools
{
   public static int toIndex(double coordinate, double resolution, int minMaxIndexXY)
   {
      return (int) Math.round(coordinate / resolution) + minMaxIndexXY;
   }

   public static double toCoordinate(int index, double resolution, int minMaxIndexXY)
   {
      return index * resolution + minMaxIndexXY;
   }
}

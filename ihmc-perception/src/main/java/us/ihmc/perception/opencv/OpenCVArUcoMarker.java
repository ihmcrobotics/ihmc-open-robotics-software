package us.ihmc.perception.opencv;

public class OpenCVArUcoMarker
{
   private final int id;
   private final double sideLength;

   public OpenCVArUcoMarker(int id, double sideLength)
   {
      this.id = id;
      this.sideLength = sideLength;
   }

   public int getId()
   {
      return id;
   }

   public double getSideLength()
   {
      return sideLength;
   }
}

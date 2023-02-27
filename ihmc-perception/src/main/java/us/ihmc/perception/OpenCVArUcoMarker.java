package us.ihmc.perception;

public class OpenCVArUcoMarker
{
   private int id;
   private double sideLength;

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

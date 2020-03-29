package us.ihmc.robotics.geometry;

public class RevisedMinMaxPointHolder
{
   double x;
   double minY;
   double maxY;

   public RevisedMinMaxPointHolder()
   {
   }

   public void set(double x, double minY, double maxY)
   {
      this.x = x;
      this.minY = minY;
      this.maxY = maxY;
   }

   public double getX()
   {
      return x;
   }

   public double getMinY()
   {
      return minY;
   }

   public double getMaxY()
   {
      return maxY;
   }
}

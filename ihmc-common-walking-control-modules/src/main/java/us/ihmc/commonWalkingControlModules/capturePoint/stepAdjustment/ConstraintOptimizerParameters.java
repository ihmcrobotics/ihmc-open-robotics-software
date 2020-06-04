package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

public class ConstraintOptimizerParameters
{
   private double maxX = 0.5;
   private double maxY = 0.5;
   private double deltaInside = 0.0;

   public void setMaxX(double maxX)
   {
      this.maxX = maxX;
   }

   public void setMaxY(double maxY)
   {
      this.maxY = maxY;
   }

   public void setDesiredDistanceInside(double distanceInside)
   {
      this.deltaInside = distanceInside;
   }

   public double getDesiredDistanceInside()
   {
      return deltaInside;
   }
   public double getMaxX()
   {
      return maxX;
   }

   public double getMaxY()
   {
      return maxY;
   }
}

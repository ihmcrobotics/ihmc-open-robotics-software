package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

public class ConstraintOptimizerParameters
{
   private double maxX = 0.5;
   private double maxY = 0.5;
   private double deltaInside = 0.0;
   private boolean constrainMaxAdjustment = true;

   private int maxIterations = 3;

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

   public void setConstrainMaxAdjustment(boolean constraintMaxAdjustment)
   {
      this.constrainMaxAdjustment = constraintMaxAdjustment;
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
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

   public boolean getConstrainMaxAdjustment()
   {
      return constrainMaxAdjustment;
   }

   public int getMaxIterations()
   {
      return maxIterations;
   }
}

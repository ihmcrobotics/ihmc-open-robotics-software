package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

public class ConstraintOptimizerParameters implements ConstraintOptimizerParametersReadOnly
{
   private double maxX = 0.5;
   private double maxY = 0.5;
   private double deltaInside = 0.0;
   private boolean constrainMaxAdjustment = true;
   private boolean shouldPerformOptimization = true;

   private boolean parametersChanged = false;

   public boolean pollParametersChanged()
   {
      if (parametersChanged)
      {
         parametersChanged = false;
         return true;
      }
      return false;
   }

   public void setMaxX(double maxX)
   {
      parametersChanged = true;
      this.maxX = maxX;
   }

   public void setMaxY(double maxY)
   {
      parametersChanged = true;
      this.maxY = maxY;
   }

   public void setDesiredDistanceInside(double distanceInside)
   {
      parametersChanged = true;
      this.deltaInside = distanceInside;
   }

   public void setConstrainMaxAdjustment(boolean constraintMaxAdjustment)
   {
      parametersChanged = true;
      this.constrainMaxAdjustment = constraintMaxAdjustment;
   }

   public void setShouldPerformOptimization(boolean shouldPerformOptimization)
   {
      this.shouldPerformOptimization = shouldPerformOptimization;
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

   public boolean shouldPerformOptimization()
   {
      return shouldPerformOptimization;
   }
}

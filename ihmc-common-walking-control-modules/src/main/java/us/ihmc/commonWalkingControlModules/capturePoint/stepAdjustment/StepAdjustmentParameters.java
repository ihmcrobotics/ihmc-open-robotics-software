package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

public abstract class StepAdjustmentParameters
{
   /**
    * Specifies the amount of ICP error (the 2D distance in XY from desired to current) that is required for the controller to consider step adjustment.
    */
   public double getMinICPErrorForStepAdjustment()
   {
      return 0.0;
   }

   /**
    * Enabling this boolean enables the use step adjustment for stabilization.
    */
   public abstract boolean allowStepAdjustment();

   /**
    * Deadband on the step adjustment.
    * When the adjustment is within the deadband, it is set to zero.
    * When it is outside the deadband, the deadband is subtracted from it.
    */
   public abstract double getAdjustmentDeadband();

   /**
    * Specifies the transfer split fraction to use for the ICP value recursion multiplier. This value is added to the time remaining
    * to compute the recursion multiplier. Increasing this value effectively causes more step adjustment to occur.
    */
   public double getTransferSplitFraction()
   {
      return 0.1;
   }

   /**
    * Specifies the maximum duration that can be included in the footstep multiplier by the {@link #getTransferSplitFraction()}.
    * This is useful when the robot by default has long split fractions.
    */
   public double maximumTimeFromTransferInFootstepMultiplier()
   {
      return 0.1;
   }

   /**
    * Specifies the minimum footstep multiplier that the robot will use to compute the desired step adjustment. This is
    * particularly useful when walking slowly or when recovering early in the, to avoid extremely large
    * footstep adjustment magnitudes.
    */
   public double getMinimumFootstepMultiplier()
   {
      return 0.33;
   }
}

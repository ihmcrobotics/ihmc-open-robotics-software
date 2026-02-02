package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

public class SplitFractionCalculatorParameters
{
   public boolean calculateSplitFractionsFromPositions()
   {
      return true;
   }

   public boolean calculateSplitFractionsFromArea()
   {
      return false;
   }

   /**
    * Default value for the transfer split fraction of the icp plan.
    */
   public double getDefaultTransferSplitFraction()
   {
      return 0.5;
   }

   /**
    * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
    */
   public double getStepHeightForLargeStepDown()
   {
      return 0.1;
   }

   /**
    * Sets the step up height for determining whether or not the transfer split fractions should be adjusted.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted so that the CoM is in a more favorable position, kind of "hanging" in the stance foot.
    */
   public double getStepHeightForLargeStepUp()
   {
      return 0.5;
   }

   /**
    * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
    */
   public double getLargestStepDownHeight()
   {
      return 0.15;
   }

   /**
    * Sets the step up height for the maximum amount of split fraction and weight distribution adjustment.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted fully, as returned by {@link #getTransferSplitFractionForStepUpAtFullDepth()} and
    * {@link #getTransferWeightDistributionForStepUpAtFullDepth()}.
    */
   public double getLargestStepUpHeight()
   {
      return 0.25;
   }

   /**
    * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired split fraction is linearly interpolated between the default value and the value returned by this function.
    */
   public double getTransferSplitFractionAtFullDepth()
   {
      return 0.3;
   }

   /**
    * Sets the desired transfer split fraction if the robot is stepping up by {@link #getLargestStepUpHeight()}.
    * If the step up height is between {@link #getStepHeightForLargeStepUp()} and {@link #getLargestStepUpHeight()}, the
    * desired split fraction is linearly interpolated between the default value and the value returned by this function.
    */
   public double getTransferSplitFractionForStepUpAtFullDepth()
   {
      return 0.6;
   }

   /**
    * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return 0.65;
   }

   /**
    * Sets the desired transfer weight distribution if the robot is stepping up by {@link #getLargestStepUpHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepUp()} and {@link #getLargestStepUpHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   public double getTransferWeightDistributionForStepUpAtFullDepth()
   {
      return 0.25;
   }

   /**
    * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   public double getTransferFinalWeightDistributionAtFullDepth()
   {
      return 0.8;
   }

   /**
    * Sets the desired transfer weight distribution if the robot is stepping up by {@link #getLargestStepUpHeight()}.
    * If the step up height is between {@link #getStepHeightForLargeStepUp()} and {@link #getLargestStepUpHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   public double getTransferFinalWeightDistributionForStepUpAtFullDepth()
   {
      return 0.35;
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
    * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
    */
   public double getFractionLoadIfFootHasFullSupport()
   {
      return 0.5;
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
    * from the foot to the midpoint, or from the midpoint to that foot.
    */
   public double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return 0.5;
   }


   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
    * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
    * this movies the midpoint CoP position to that foot.
    */
   public double getFractionLoadIfOtherFootHasNoWidth()
   {
      return 0.5;
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
    * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
    */
   public double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return 0.5;
   }
}

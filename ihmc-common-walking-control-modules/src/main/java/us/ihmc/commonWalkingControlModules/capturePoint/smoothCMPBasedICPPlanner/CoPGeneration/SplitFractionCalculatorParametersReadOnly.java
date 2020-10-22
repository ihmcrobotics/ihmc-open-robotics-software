package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;

public interface SplitFractionCalculatorParametersReadOnly
{
   boolean calculateSplitFractionsFromPositions();

   boolean calculateSplitFractionsFromArea();

   /**
    * Default value for the transfer split fraction of the icp plan.
    * @see ICPPlannerParameters#getTransferSplitFraction() 
    */
   double getDefaultTransferSplitFraction();

   /**
    * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
    */
   double getStepHeightForLargeStepDown();

   /**
    * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
    */
   double getLargestStepDownHeight();

   /**
    * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired split fraction is linearly interpolated between the default value and the value returned by this function.
    */
   double getTransferSplitFractionAtFullDepth();

   /**
    * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   double getTransferWeightDistributionAtFullDepth();

   /**
    * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   default double getTransferFinalWeightDistributionAtFullDepth()
   {
      return getTransferWeightDistributionAtFullDepth();
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
    * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
    */
   double getFractionLoadIfFootHasFullSupport();

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
    * from the foot to the midpoint, or from the midpoint to that foot.
    */
   double getFractionTimeOnFootIfFootHasFullSupport();

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
    * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
    * this movies the midpoint CoP position to that foot.
    */
   double getFractionLoadIfOtherFootHasNoWidth();

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
    * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
    */
   double getFractionTimeOnFootIfOtherFootHasNoWidth();
}

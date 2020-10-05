package us.ihmc.footstepPlanning.icp;

import controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

public interface SplitFractionCalculatorParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Default value for the transfer split fraction of the icp plan.
    * @see ICPPlannerParameters#getTransferSplitFraction() 
    */
   default double getDefaultTransferSplitFraction()
   {
      return get(SplitFractionCalculatorParameterKeys.defaultTransferSplitFraction);
   }

   /**
    * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
    */
   default double getStepHeightForLargeStepDown()
   {
      return get(SplitFractionCalculatorParameterKeys.stepHeightForLargeStepDown);
   }

   /**
    * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
    */
   default double getLargestStepDownHeight()
   {
      return get(SplitFractionCalculatorParameterKeys.largestStepDownHeight);
   }

   /**
    * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired split fraction is linearly interpolated between the default value and the value returned by this function.
    */
   default double getTransferSplitFractionAtFullDepth()
   {
      return get(SplitFractionCalculatorParameterKeys.transferSplitFractionAtFullDepth);
   }

   /**
    * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   default double getTransferWeightDistributionAtFullDepth()
   {
      return get(SplitFractionCalculatorParameterKeys.transferWeightDistributionAtFullDepth);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
    * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
    */
   default double getFractionLoadIfFootHasFullSupport()
   {
      return get(SplitFractionCalculatorParameterKeys.fractionLoadIfFootHasFullSupport);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
    * from the foot to the midpoint, or from the midpoint to that foot.
    */
   default double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return get(SplitFractionCalculatorParameterKeys.fractionTimeOnFootIfFootHasFullSupport);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
    * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
    * this movies the midpoint CoP position to that foot.
    */
   default double getFractionLoadIfOtherFootHasNoWidth()
   {
      return get(SplitFractionCalculatorParameterKeys.fractionLoadIfOtherFootHasNoWidth);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
    * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
    */
   default double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return get(SplitFractionCalculatorParameterKeys.fractionTimeOnFootIfOtherFootHasNoWidth);
   }

   default SplitFractionCalculatorParametersPacket getAsPacket()
   {
      SplitFractionCalculatorParametersPacket packet = new SplitFractionCalculatorParametersPacket();

      packet.setDefaultTransferSplitFraction(getDefaultTransferSplitFraction());
      packet.setStepHeightForLargeStepDown(getStepHeightForLargeStepDown());
      packet.setLargestStepDownHeight(getLargestStepDownHeight());
      packet.setTransferSplitFractionAtFullDepth(getTransferSplitFractionAtFullDepth());
      packet.setTransferWeightDistributionAtFullDepth(getTransferWeightDistributionAtFullDepth());

      packet.setFractionLoadIfFootHasFullSupport(getFractionLoadIfFootHasFullSupport());
      packet.setFractionTimeOnFootIfFootHasFullSupport(getFractionTimeOnFootIfFootHasFullSupport());
      packet.setFractionLoadIfOtherFootHasNoWidth(getFractionLoadIfOtherFootHasNoWidth());
      packet.setFractionTimeOnFootIfOtherFootHasNoWidth(getFractionTimeOnFootIfOtherFootHasNoWidth());

      return packet;
   }
}

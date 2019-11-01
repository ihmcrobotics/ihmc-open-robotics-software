package us.ihmc.footstepPlanning.postProcessing.parameters;

import controller_msgs.msg.dds.FootstepPostProcessingParametersPacket;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

public interface FootstepPostProcessingParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Determines whether the post processing module for adjusting the split fractions based on the footstep positions for the CoP trajectory is enabled.
    */
   default boolean positionSplitFractionProcessingEnabled()
   {
      return get(FootstepPostProcessingKeys.positionSplitFractionProcessingEnabled);
   }

   /**
    * Determines whether the post processing module for adjusting the split fractions based on the foothold areas for the CoP trajectory is enabled.
    */
   default boolean areaSplitFractionProcessingEnabled()
   {
      return get(FootstepPostProcessingKeys.areaSplitFractionProcessingEnabled);
   }

   /**
    * Determines whether the post processing module for swinging over planar regions is enabled.
    */
   default boolean swingOverRegionsProcessingEnabled()
   {
      return get(FootstepPostProcessingKeys.swingOverRegionsProcessingEnabled);
   }

   /**
    * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
    */
   default double getStepHeightForLargeStepDown()
   {
      return get(FootstepPostProcessingKeys.stepHeightForLargeStepDown);
   }

   /**
    * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
    * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
    * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
    */
   default double getLargestStepDownHeight()
   {
      return get(FootstepPostProcessingKeys.largestStepDownHeight);
   }

   /**
    * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired split fraction is linearly interpolated between the default value and the value returned by this function.
    */
   default double getTransferSplitFractionAtFullDepth()
   {
      return get(FootstepPostProcessingKeys.transferSplitFractionAtFullDepth);
   }

   /**
    * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
    * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
    * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
    */
   default double getTransferWeightDistributionAtFullDepth()
   {
      return get(FootstepPostProcessingKeys.transferWeightDistributionAtFullDepth);
   }

   /**
    * If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
    * along the swing foot trajectory and the planar regions in the environment.
    */
   default double getMinimumSwingFootClearance()
   {
      return get(FootstepPostProcessingKeys.minimumSwingFootClearance);
   }

   default boolean getDoInitialFastApproximation()
   {
      return get(FootstepPostProcessingKeys.doInitialFastApproximation);
   }

   /**
    * If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
    */
   default int getNumberOfChecksPerSwing()
   {
      return get(FootstepPostProcessingKeys.numberOfChecksPerSwing);
   }

   /**
    * If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
    * collisions with the environment.
    */
   default int getMaximumNumberOfAdjustmentAttempts()
   {
      return get(FootstepPostProcessingKeys.maximumNumberOfAdjustmentAttempts);
   }

   /**
    * If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
    */
   default double getMaximumWaypointAdjustmentDistance()
   {
      return get(FootstepPostProcessingKeys.maximumWaypointAdjustmentDistance);
   }

   /**
    * If using the swing over planar regions module, this is the distance that the swing waypoints will be adjusted by.
    */
   default double getIncrementalWaypointAdjustmentDistance()
   {
      return get(FootstepPostProcessingKeys.incrementalWaypointAdjustmentDistance);
   }

   default double getMinimumHeightAboveFloorForCollision()
   {
      return get(FootstepPostProcessingKeys.minimumHeightAboveFloorForCollision);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
    * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
    */
   default double getFractionLoadIfFootHasFullSupport()
   {
      return get(FootstepPostProcessingKeys.fractionLoadIfFootHasFullSupport);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
    * from the foot to the midpoint, or from the midpoint to that foot.
    */
   default double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return get(FootstepPostProcessingKeys.fractionTimeOnFootIfFootHasFullSupport);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
    * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
    * this movies the midpoint CoP position to that foot.
    */
   default double getFractionLoadIfOtherFootHasNoWidth()
   {
      return get(FootstepPostProcessingKeys.fractionLoadIfOtherFootHasNoWidth);
   }

   /**
    * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
    * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
    * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
    */
   default double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return get(FootstepPostProcessingKeys.fractionTimeOnFootIfOtherFootHasNoWidth);
   }

   default FootstepPostProcessingParametersPacket getAsPacket()
   {
      FootstepPostProcessingParametersPacket packet = new FootstepPostProcessingParametersPacket();

      packet.setPositionSplitFractionProcessingEnabled(positionSplitFractionProcessingEnabled());
      packet.setAreaSplitFractionProcessingEnabled(areaSplitFractionProcessingEnabled());
      packet.setSwingOverRegionsProcessingEnabled(swingOverRegionsProcessingEnabled());

      packet.setStepHeightForLargeStepDown(getStepHeightForLargeStepDown());
      packet.setLargestStepDownHeight(getLargestStepDownHeight());
      packet.setTransferSplitFractionAtFullDepth(getTransferSplitFractionAtFullDepth());
      packet.setTransferWeightDistributionAtFullDepth(getTransferWeightDistributionAtFullDepth());

      packet.setMinimumSwingFootClearance(getMinimumSwingFootClearance());
      packet.setDoInitialFastApproximation(getDoInitialFastApproximation());
      packet.setNumberOfChecksPerSwing(getNumberOfChecksPerSwing());
      packet.setMaximumNumberOfAdjustmentAttempts(getMaximumNumberOfAdjustmentAttempts());
      packet.setMaximumWaypointAdjustmentDistance(getMaximumWaypointAdjustmentDistance());
      packet.setIncrementalWaypointAdjustmentDistance(getIncrementalWaypointAdjustmentDistance());
      packet.setMinimumHeightAboveFloorForCollision(getMinimumHeightAboveFloorForCollision());

      packet.setFractionLoadIfFootHasFullSupport(getFractionLoadIfFootHasFullSupport());
      packet.setFractionTimeOnFootIfFootHasFullSupport(getFractionTimeOnFootIfFootHasFullSupport());
      packet.setFractionLoadIfOtherFootHasNoWidth(getFractionLoadIfOtherFootHasNoWidth());
      packet.setFractionTimeOnFootIfOtherFootHasNoWidth(getFractionTimeOnFootIfOtherFootHasNoWidth());

      return packet;
   }
}

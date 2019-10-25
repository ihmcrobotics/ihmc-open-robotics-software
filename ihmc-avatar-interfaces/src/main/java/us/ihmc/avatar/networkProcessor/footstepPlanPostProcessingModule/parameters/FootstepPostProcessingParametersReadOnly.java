package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

public interface FootstepPostProcessingParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Determines whether the post processing module for adjusting the split fractions for the CoP trajectory is enabled.
    */
   default boolean splitFractionProcessingEnabled()
   {
      return get(FootstepPostProcessingKeys.splitFractionProcessingEnabled);
   }

   /**
    * Determines whether the post processing module for swinging over planar regions is enabled.
    */
   default boolean swingOverRegionsEnabled()
   {
      return get(FootstepPostProcessingKeys.swingOverRegionsEnabled);
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
}

package us.ihmc.footstepPlanning.postProcessing.parameters;

import controller_msgs.msg.dds.FootstepPostProcessingParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface FootstepPostProcessingParametersBasics extends FootstepPostProcessingParametersReadOnly, StoredPropertySetBasics
{
   default void set(FootstepPostProcessingParametersReadOnly footstepPostProcessingParameters)
   {
      setAll(footstepPostProcessingParameters.getAll());
   }

   default void setSplitFractionProcessingEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.splitFractionProcessingEnabled, enabled);
   }

   default void setSwingOverRegionsEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.swingOverRegionsEnabled, enabled);
   }

   default void setStepHeightForLargeStepDown(double height)
   {
      set(FootstepPostProcessingKeys.stepHeightForLargeStepDown, height);
   }

   default void setLargestStepDownHeight(double height)
   {
      set(FootstepPostProcessingKeys.largestStepDownHeight, height);
   }

   default void setTransferSplitFractionAtFullDepth(double splitFraction)
   {
      set(FootstepPostProcessingKeys.transferSplitFractionAtFullDepth, splitFraction);
   }

   default void setTransferWeightDistributionAtFullDepth(double weightDistribution)
   {
      set(FootstepPostProcessingKeys.transferWeightDistributionAtFullDepth, weightDistribution);
   }

   default void setMinimumSwingFootClearance(double minimumSwingFootClearance)
   {
      set(FootstepPostProcessingKeys.minimumSwingFootClearance, minimumSwingFootClearance);
   }

   default void setNumberOfChecksPerSwing(int numberOfChecksPerSwing)
   {
      set(FootstepPostProcessingKeys.numberOfChecksPerSwing, numberOfChecksPerSwing);
   }

   default void setMaximumNumberOfAdjustmentAttempts(int numberOfAttempts)
   {
      set(FootstepPostProcessingKeys.maximumNumberOfAdjustmentAttempts, numberOfAttempts);
   }

   default void setMaximumWaypointAdjustmentDistance(double distance)
   {
      set(FootstepPostProcessingKeys.maximumWaypointAdjustmentDistance, distance);
   }

   default void setIncrementalWaypointAdjustmentDistance(double distance)
   {
      set(FootstepPostProcessingKeys.incrementalWaypointAdjustmentDistance, distance);
   }

   default void set(FootstepPostProcessingParametersPacket packet)
   {
      setSplitFractionProcessingEnabled(packet.getSplitFractionProcessingEnabled());
      setSwingOverRegionsEnabled(packet.getSwingOverRegionsEnabled());

      if (packet.getStepHeightForLargeStepDown() != -1.0)
         setStepHeightForLargeStepDown(packet.getStepHeightForLargeStepDown());
      if (packet.getLargestStepDownHeight() != -1.0)
         setLargestStepDownHeight(packet.getLargestStepDownHeight());
      if (packet.getTransferSplitFractionAtFullDepth() != -1.0)
         setTransferSplitFractionAtFullDepth(packet.getTransferSplitFractionAtFullDepth());
      if (packet.getTransferWeightDistributionAtFullDepth() != -1.0)
         setTransferWeightDistributionAtFullDepth(packet.getTransferWeightDistributionAtFullDepth());

      if (packet.getMinimumSwingFootClearance() != -1.0)
         setMinimumSwingFootClearance(packet.getMinimumSwingFootClearance());
      if (packet.getMaximumWaypointAdjustmentDistance() != -1.0)
         setMaximumWaypointAdjustmentDistance(packet.getMaximumWaypointAdjustmentDistance());
      if (packet.getIncrementalWaypointAdjustmentDistance() != -1.0)
         setIncrementalWaypointAdjustmentDistance(packet.getIncrementalWaypointAdjustmentDistance());
      setNumberOfChecksPerSwing((int) packet.getNumberOfChecksPerSwing());
      setMaximumNumberOfAdjustmentAttempts((int) packet.getMaximumNumberOfAdjustmentAttempts());
   }
}

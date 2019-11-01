package us.ihmc.footstepPlanning.postProcessing.parameters;

import controller_msgs.msg.dds.FootstepPostProcessingParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface FootstepPostProcessingParametersBasics extends FootstepPostProcessingParametersReadOnly, StoredPropertySetBasics
{
   default void set(FootstepPostProcessingParametersReadOnly footstepPostProcessingParameters)
   {
      setAll(footstepPostProcessingParameters.getAll());
   }

   default void setAreaSplitFractionProcessingEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.areaSplitFractionProcessingEnabled, enabled);
   }

   default void setPositionSplitFractionProcessingEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.positionSplitFractionProcessingEnabled, enabled);
   }

   default void setSwingOverRegionsProcessingEnabled(boolean enabled)
   {
      set(FootstepPostProcessingKeys.swingOverRegionsProcessingEnabled, enabled);
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

   default void setDoInitialFastApproximation(boolean doApproximation)
   {
      set(FootstepPostProcessingKeys.doInitialFastApproximation, doApproximation);
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

   default void setMinimumHeightAboveFloorForCollision(double height)
   {
      set(FootstepPostProcessingKeys.minimumHeightAboveFloorForCollision, height);
   }

   default void setFractionLoadIfFootHasFullSupport(double fraction)
   {
      set(FootstepPostProcessingKeys.fractionLoadIfFootHasFullSupport, fraction);
   }

   default void setFractionTimeOnFootIfFootHasFullSupport(double fraction)
   {
      set(FootstepPostProcessingKeys.fractionTimeOnFootIfFootHasFullSupport, fraction);
   }

   default void setFractionLoadIfOtherFootHasNoWidth(double fraction)
   {
      set(FootstepPostProcessingKeys.fractionLoadIfOtherFootHasNoWidth, fraction);
   }

   default void setFractionTimeOnFootIfOtherFootHasNoWidth(double fraction)
   {
      set(FootstepPostProcessingKeys.fractionTimeOnFootIfOtherFootHasNoWidth, fraction);
   }

   default void set(FootstepPostProcessingParametersPacket packet)
   {
      setPositionSplitFractionProcessingEnabled(packet.getPositionSplitFractionProcessingEnabled());
      setAreaSplitFractionProcessingEnabled(packet.getAreaSplitFractionProcessingEnabled());
      setSwingOverRegionsProcessingEnabled(packet.getSwingOverRegionsProcessingEnabled());

      if (packet.getStepHeightForLargeStepDown() != -1.0)
         setStepHeightForLargeStepDown(packet.getStepHeightForLargeStepDown());
      if (packet.getLargestStepDownHeight() != -1.0)
         setLargestStepDownHeight(packet.getLargestStepDownHeight());
      if (packet.getTransferSplitFractionAtFullDepth() != -1.0)
         setTransferSplitFractionAtFullDepth(packet.getTransferSplitFractionAtFullDepth());
      if (packet.getTransferWeightDistributionAtFullDepth() != -1.0)
         setTransferWeightDistributionAtFullDepth(packet.getTransferWeightDistributionAtFullDepth());

      setDoInitialFastApproximation(packet.getDoInitialFastApproximation());
      if (packet.getMinimumSwingFootClearance() != -1.0)
         setMinimumSwingFootClearance(packet.getMinimumSwingFootClearance());
      if (packet.getMaximumWaypointAdjustmentDistance() != -1.0)
         setMaximumWaypointAdjustmentDistance(packet.getMaximumWaypointAdjustmentDistance());
      if (packet.getIncrementalWaypointAdjustmentDistance() != -1.0)
         setIncrementalWaypointAdjustmentDistance(packet.getIncrementalWaypointAdjustmentDistance());
      if (packet.getMinimumHeightAboveFloorForCollision() != -1.0)
         setMinimumHeightAboveFloorForCollision(packet.getMinimumHeightAboveFloorForCollision());
      setNumberOfChecksPerSwing((int) packet.getNumberOfChecksPerSwing());
      setMaximumNumberOfAdjustmentAttempts((int) packet.getMaximumNumberOfAdjustmentAttempts());

      if (packet.getFractionLoadIfFootHasFullSupport() != -1.0)
         setFractionLoadIfFootHasFullSupport(packet.getFractionLoadIfFootHasFullSupport());
      if (packet.getFractionTimeOnFootIfFootHasFullSupport() != -1.0)
         setFractionTimeOnFootIfFootHasFullSupport(packet.getFractionTimeOnFootIfFootHasFullSupport());
      if (packet.getFractionLoadIfOtherFootHasNoWidth() != -1.0)
         setFractionLoadIfOtherFootHasNoWidth(packet.getFractionLoadIfOtherFootHasNoWidth());
      if (packet.getFractionTimeOnFootIfOtherFootHasNoWidth() != -1.0)
         setFractionTimeOnFootIfOtherFootHasNoWidth(packet.getFractionTimeOnFootIfOtherFootHasNoWidth());
   }
}

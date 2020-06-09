package us.ihmc.footstepPlanning.swing;

import controller_msgs.msg.dds.SwingPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface SwingPlannerParametersBasics extends SwingPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void set(SwingPlannerParametersReadOnly swingPlannerParameters)
   {
      setAll(swingPlannerParameters.getAll());
   }

   default void setMinimumSwingHeight(double minimumSwingHeight)
   {
      set(SwingPlannerParameterKeys.minimumSwingHeight, minimumSwingHeight);
   }

   default void setMaximumSwingHeight(double maximumSwingHeight)
   {
      set(SwingPlannerParameterKeys.maximumSwingHeight, maximumSwingHeight);
   }

   default void setMaximumStepHeightForMinimumSwingHeight(double maximumStepHeightForMinimumSwingHeight)
   {
      set(SwingPlannerParameterKeys.maximumStepHeightForMinimumSwingHeight, maximumStepHeightForMinimumSwingHeight);
   }

   default void setMinimumStepHeightForMaximumSwingHeight(double minimumStepHeightForMaximumSwingHeight)
   {
      set(SwingPlannerParameterKeys.minimumStepHeightForMaximumSwingHeight, minimumStepHeightForMaximumSwingHeight);
   }

   default void setMinimumSwingTime(double minimumSwingTime)
   {
      set(SwingPlannerParameterKeys.minimumSwingTime, minimumSwingTime);
   }

   default void setMaximumSwingTime(double maximumSwingTime)
   {
      set(SwingPlannerParameterKeys.maximumSwingTime, maximumSwingTime);
   }

   default void setMaximumStepTranslationForMinimumSwingTime(double maximumStepTranslationForMinimumSwingTime)
   {
      set(SwingPlannerParameterKeys.maximumStepTranslationForMinimumSwingTime, maximumStepTranslationForMinimumSwingTime);
   }

   default void setMinimumStepTranslationForMaximumSwingTime(double minimumStepTranslationForMaximumSwingTime)
   {
      set(SwingPlannerParameterKeys.minimumStepTranslationForMaximumSwingTime, minimumStepTranslationForMaximumSwingTime);
   }

   default void setMaximumStepHeightForMinimumSwingTime(double maximumStepHeightForMinimumSwingTime)
   {
      set(SwingPlannerParameterKeys.maximumStepHeightForMinimumSwingTime, maximumStepHeightForMinimumSwingTime);
   }

   default void setMinimumStepHeightForMaximumSwingTime(double minimumStepHeightForMaximumSwingTime)
   {
      set(SwingPlannerParameterKeys.minimumStepHeightForMaximumSwingTime, minimumStepHeightForMaximumSwingTime);
   }

   default void setFootStubClearance(double footStubClearance)
   {
      set(SwingPlannerParameterKeys.footStubClearance, footStubClearance);
   }

   default void setWaypointProportionShiftForStubAvoidance(double waypointProportionShiftForStubAvoidance)
   {
      set(SwingPlannerParameterKeys.waypointProportionShiftForStubAvoidance, waypointProportionShiftForStubAvoidance);
   }

   default void setDoInitialFastApproximation(boolean doInitialFastApproximation)
   {
      set(SwingPlannerParameterKeys.doInitialFastApproximation, doInitialFastApproximation);
   }

   default void setMinimumSwingFootClearance(double minimumSwingFootClearance)
   {
      set(SwingPlannerParameterKeys.minimumSwingFootClearance, minimumSwingFootClearance);
   }

   default void setFastApproximationLessClearance(double fastApproximationLessClearance)
   {
      set(SwingPlannerParameterKeys.fastApproximationLessClearance, fastApproximationLessClearance);
   }

   default void setNumberOfChecksPerSwing(int numberOfChecksPerSwing)
   {
      set(SwingPlannerParameterKeys.numberOfChecksPerSwing, numberOfChecksPerSwing);
   }

   default void setMaximumNumberOfAdjustmentAttempts(int maximumNumberOfAdjustmentAttempts)
   {
      set(SwingPlannerParameterKeys.maximumNumberOfAdjustmentAttempts, maximumNumberOfAdjustmentAttempts);
   }

   default void setMaximumWaypointAdjustmentDistance(double maximumWaypointAdjustmentDistance)
   {
      set(SwingPlannerParameterKeys.maximumWaypointAdjustmentDistance, maximumWaypointAdjustmentDistance);
   }

   default void setMinimumAdjustmentIncrementDistance(double minimumAdjustmentIncrementDistance)
   {
      set(SwingPlannerParameterKeys.minimumAdjustmentIncrementDistance, minimumAdjustmentIncrementDistance);
   }

   default void setMaximumAdjustmentIncrementDistance(double maximumAdjustmentIncrementDistance)
   {
      set(SwingPlannerParameterKeys.maximumAdjustmentIncrementDistance, maximumAdjustmentIncrementDistance);
   }

   default void setAdjustmentIncrementDistanceGain(double adjustmentIncrementDistanceGain)
   {
      set(SwingPlannerParameterKeys.adjustmentIncrementDistanceGain, adjustmentIncrementDistanceGain);
   }

   default void setMinimumHeightAboveFloorForCollision(double minimumHeightAboveFloorForCollision)
   {
      set(SwingPlannerParameterKeys.minimumHeightAboveFloorForCollision, minimumHeightAboveFloorForCollision);
   }

   default void setAdditionalSwingTimeIfExpanded(double additionalSwingTimeIfExpanded)
   {
      set(SwingPlannerParameterKeys.additionalSwingTimeIfExpanded, additionalSwingTimeIfExpanded);
   }

   default void set(SwingPlannerParametersPacket packet)
   {
      setDoInitialFastApproximation(packet.getDoInitialFastApproximation());
      setNumberOfChecksPerSwing((int) packet.getNumberOfChecksPerSwing());
      setMaximumNumberOfAdjustmentAttempts((int) packet.getMaximumNumberOfAdjustmentAttempts());

      if (packet.getMinimumSwingHeight() != -1.0)
         setMinimumSwingHeight(packet.getMinimumSwingHeight());
      if (packet.getMaximumSwingHeight() != -1.0)
         setMaximumSwingHeight(packet.getMaximumSwingHeight());
      if (packet.getMaximumStepHeightForMinimumSwingHeight() != -1.0)
         setMaximumStepHeightForMinimumSwingHeight(packet.getMaximumStepHeightForMinimumSwingHeight());
      if (packet.getMinimumStepHeightForMaximumSwingHeight() != -1.0)
         setMinimumStepHeightForMaximumSwingHeight(packet.getMinimumStepHeightForMaximumSwingHeight());
      if (packet.getMinimumSwingTime() != -1.0)
         setMinimumSwingTime(packet.getMinimumSwingTime());
      if (packet.getMaximumSwingTime() != -1.0)
         setMaximumSwingTime(packet.getMaximumSwingTime());
      if (packet.getMaximumStepTranslationForMinimumSwingTime() != -1.0)
         setMaximumStepTranslationForMinimumSwingTime(packet.getMaximumStepTranslationForMinimumSwingTime());
      if (packet.getMinimumStepTranslationForMaximumSwingTime() != -1.0)
         setMinimumStepTranslationForMaximumSwingTime(packet.getMinimumStepTranslationForMaximumSwingTime());
      if (packet.getMaximumStepHeightForMinimumSwingTime() != -1.0)
         setMaximumStepHeightForMinimumSwingTime(packet.getMaximumStepHeightForMinimumSwingTime());
      if (packet.getMinimumStepHeightForMaximumSwingTime() != -1.0)
         setMinimumStepHeightForMaximumSwingTime(packet.getMinimumStepHeightForMaximumSwingTime());
      if (packet.getFootStubClearance() != -1.0)
         setFootStubClearance(packet.getFootStubClearance());
      if (packet.getWaypointProportionShiftForStubAvoidance() != -1.0)
         setWaypointProportionShiftForStubAvoidance(packet.getWaypointProportionShiftForStubAvoidance());
      if (packet.getMinimumSwingFootClearance() != -1.0)
         setMinimumSwingFootClearance(packet.getMinimumSwingFootClearance());
      if (packet.getFastApproximationLessClearance() != -1.0)
         setFastApproximationLessClearance(packet.getFastApproximationLessClearance());
      if (packet.getMaximumWaypointAdjustmentDistance() != -1.0)
         setMaximumWaypointAdjustmentDistance(packet.getMaximumWaypointAdjustmentDistance());
      if (packet.getMinimumAdjustmentIncrementDistance() != -1.0)
         setMinimumAdjustmentIncrementDistance(packet.getMinimumAdjustmentIncrementDistance());
      if (packet.getMaximumAdjustmentIncrementDistance() != -1.0)
         setMaximumAdjustmentIncrementDistance(packet.getMaximumAdjustmentIncrementDistance());
      if (packet.getAdjustmentIncrementDistanceGain() != -1.0)
         setAdjustmentIncrementDistanceGain(packet.getAdjustmentIncrementDistanceGain());
      if (packet.getMinimumHeightAboveFloorForCollision() != -1.0)
         setMinimumHeightAboveFloorForCollision(packet.getMinimumHeightAboveFloorForCollision());
      if (packet.getAdditionalSwingTimeIfExpanded() != -1.0)
         setAdditionalSwingTimeIfExpanded(packet.getAdditionalSwingTimeIfExpanded());
   }
}

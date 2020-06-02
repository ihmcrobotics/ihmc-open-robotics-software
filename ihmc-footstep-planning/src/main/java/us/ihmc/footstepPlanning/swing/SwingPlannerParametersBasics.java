package us.ihmc.footstepPlanning.swing;

import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingKeys;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface SwingPlannerParametersBasics extends SwingPlannerParametersReadOnly, StoredPropertySetBasics
{
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
      set(FootstepPostProcessingKeys.minimumSwingFootClearance, minimumSwingFootClearance);
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
}

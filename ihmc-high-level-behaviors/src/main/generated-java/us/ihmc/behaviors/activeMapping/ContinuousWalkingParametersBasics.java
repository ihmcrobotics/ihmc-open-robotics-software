package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousWalkingParametersBasics extends ContinuousWalkingParametersReadOnly, StoredPropertySetBasics
{
   default void setEnableContinuousWalking(boolean enableContinuousWalking)
   {
      set(ContinuousWalkingParameters.enableContinuousWalking, enableContinuousWalking);
   }

   default void setShortcutIsPressed(boolean shortcutIsPressed)
   {
      set(ContinuousWalkingParameters.shortcutIsPressed, shortcutIsPressed);
   }

   default void setStepPublisherEnabled(boolean stepPublisherEnabled)
   {
      set(ContinuousWalkingParameters.stepPublisherEnabled, stepPublisherEnabled);
   }

   default void setOverrideEntireQueueEachStep(boolean overrideEntireQueueEachStep)
   {
      set(ContinuousWalkingParameters.overrideEntireQueueEachStep, overrideEntireQueueEachStep);
   }

   default void setNumberOfStepsToSend(int numberOfStepsToSend)
   {
      set(ContinuousWalkingParameters.numberOfStepsToSend, numberOfStepsToSend);
   }

   default void setGoalPoseForwardDistance(double goalPoseForwardDistance)
   {
      set(ContinuousWalkingParameters.goalPoseForwardDistance, goalPoseForwardDistance);
   }

   default void setGoalPoseUpDistance(double goalPoseUpDistance)
   {
      set(ContinuousWalkingParameters.goalPoseUpDistance, goalPoseUpDistance);
   }

   default void setSwingTime(double swingTime)
   {
      set(ContinuousWalkingParameters.swingTime, swingTime);
   }

   default void setTransferTime(double transferTime)
   {
      set(ContinuousWalkingParameters.transferTime, transferTime);
   }

   default void setPlannerTimeoutFraction(double plannerTimeoutFraction)
   {
      set(ContinuousWalkingParameters.plannerTimeoutFraction, plannerTimeoutFraction);
   }

   default void setPlanningWithoutReferenceTimeout(double planningWithoutReferenceTimeout)
   {
      set(ContinuousWalkingParameters.planningWithoutReferenceTimeout, planningWithoutReferenceTimeout);
   }

   default void setLogFootstepPlans(boolean logFootstepPlans)
   {
      set(ContinuousWalkingParameters.logFootstepPlans, logFootstepPlans);
   }

   default void setDefaultOperatingMode(boolean defaultOperatingMode)
   {
      set(ContinuousWalkingParameters.defaultOperatingMode, defaultOperatingMode);
   }

   default void setDisableUpdatingHeightMap(boolean disableUpdatingHeightMap)
   {
      set(ContinuousWalkingParameters.disableUpdatingHeightMap, disableUpdatingHeightMap);
   }
}

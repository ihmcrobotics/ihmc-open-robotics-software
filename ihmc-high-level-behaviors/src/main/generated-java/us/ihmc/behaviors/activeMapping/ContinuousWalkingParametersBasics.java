package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousWalkingParametersBasics extends ContinuousWalkingParametersReadOnly, StoredPropertySetBasics
{
   default void setContinuousWalkingEnabled(boolean continuousWalkingEnabled)
   {
      set(ContinuousWalkingParameters.continuousWalkingEnabled, continuousWalkingEnabled);
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

   default void setPlanningTimeoutFraction(double planningTimeoutFraction)
   {
      set(ContinuousWalkingParameters.planningTimeoutFraction, planningTimeoutFraction);
   }

   default void setPlanningReferenceTimeout(double planningReferenceTimeout)
   {
      set(ContinuousWalkingParameters.planningReferenceTimeout, planningReferenceTimeout);
   }

   default void setPlanningWithoutReferenceTimeout(double planningWithoutReferenceTimeout)
   {
      set(ContinuousWalkingParameters.planningWithoutReferenceTimeout, planningWithoutReferenceTimeout);
   }

   default void setLogFootstepPlans(boolean logFootstepPlans)
   {
      set(ContinuousWalkingParameters.logFootstepPlans, logFootstepPlans);
   }
}

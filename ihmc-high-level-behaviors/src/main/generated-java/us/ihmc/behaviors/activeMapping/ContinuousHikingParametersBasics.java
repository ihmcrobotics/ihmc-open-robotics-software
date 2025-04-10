package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousHikingParametersBasics extends ContinuousHikingParametersReadOnly, StoredPropertySetBasics
{
   default void setStepPublisherEnabled(boolean stepPublisherEnabled)
   {
      set(ContinuousHikingParameters.stepPublisherEnabled, stepPublisherEnabled);
   }

   default void setOverrideEntireQueueEachStep(boolean overrideEntireQueueEachStep)
   {
      set(ContinuousHikingParameters.overrideEntireQueueEachStep, overrideEntireQueueEachStep);
   }

   default void setNumberOfStepsToSend(int numberOfStepsToSend)
   {
      set(ContinuousHikingParameters.numberOfStepsToSend, numberOfStepsToSend);
   }

   default void setGoalPoseForwardDistance(double goalPoseForwardDistance)
   {
      set(ContinuousHikingParameters.goalPoseForwardDistance, goalPoseForwardDistance);
   }

   default void setGoalPoseUpDistance(double goalPoseUpDistance)
   {
      set(ContinuousHikingParameters.goalPoseUpDistance, goalPoseUpDistance);
   }

   default void setGoalPoseBackwardDistance(double goalPoseBackwardDistance)
   {
      set(ContinuousHikingParameters.goalPoseBackwardDistance, goalPoseBackwardDistance);
   }

   default void setGoalPoseSidewaysDistance(double goalPoseSidewaysDistance)
   {
      set(ContinuousHikingParameters.goalPoseSidewaysDistance, goalPoseSidewaysDistance);
   }

   default void setSwingTime(double swingTime)
   {
      set(ContinuousHikingParameters.swingTime, swingTime);
   }

   default void setTransferTime(double transferTime)
   {
      set(ContinuousHikingParameters.transferTime, transferTime);
   }

   default void setPlanningWithoutReferenceTimeout(double planningWithoutReferenceTimeout)
   {
      set(ContinuousHikingParameters.planningWithoutReferenceTimeout, planningWithoutReferenceTimeout);
   }

   default void setPercentThroughSwingToStartPlanning(double percentThroughSwingToStartPlanning)
   {
      set(ContinuousHikingParameters.percentThroughSwingToStartPlanning, percentThroughSwingToStartPlanning);
   }

   default void setLogFootstepPlans(boolean logFootstepPlans)
   {
      set(ContinuousHikingParameters.logFootstepPlans, logFootstepPlans);
   }

   default void setNextWaypointDistanceMargin(double nextWaypointDistanceMargin)
   {
      set(ContinuousHikingParameters.nextWaypointDistanceMargin, nextWaypointDistanceMargin);
   }

   default void setNinetyDegreeTurnSwingTime(double ninetyDegreeTurnSwingTime)
   {
      set(ContinuousHikingParameters.ninetyDegreeTurnSwingTime, ninetyDegreeTurnSwingTime);
   }

   default void setNinetyDegreeTurnTransferTime(double ninetyDegreeTurnTransferTime)
   {
      set(ContinuousHikingParameters.ninetyDegreeTurnTransferTime, ninetyDegreeTurnTransferTime);
   }
}

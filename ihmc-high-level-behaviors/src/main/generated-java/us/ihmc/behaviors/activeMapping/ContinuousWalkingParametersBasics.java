package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousWalkingParametersBasics extends ContinuousWalkingParametersReadOnly, StoredPropertySetBasics
{
   default void setActiveMapping(boolean activeMapping)
   {
      set(ContinuousWalkingParameters.activeMapping, activeMapping);
   }

   default void setClearEntireControllerQueue(boolean clearEntireControllerQueue)
   {
      set(ContinuousWalkingParameters.clearEntireControllerQueue, clearEntireControllerQueue);
   }

   default void setNumberOfStepsToSend(int numberOfStepsToSend)
   {
      set(ContinuousWalkingParameters.numberOfStepsToSend, numberOfStepsToSend);
   }

   default void setMaxStepsToHoldInControllerQueue(int maxStepsToHoldInControllerQueue)
   {
      set(ContinuousWalkingParameters.maxStepsToHoldInControllerQueue, maxStepsToHoldInControllerQueue);
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

   default void setPlanningWithReferenceTimeout(double planningWithReferenceTimeout)
   {
      set(ContinuousWalkingParameters.planningWithReferenceTimeout, planningWithReferenceTimeout);
   }

   default void setInitialPlanningTimeout(double initialPlanningTimeout)
   {
      set(ContinuousWalkingParameters.initialPlanningTimeout, initialPlanningTimeout);
   }
}

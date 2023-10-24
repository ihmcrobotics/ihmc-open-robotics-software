package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousPlanningParametersBasics extends ContinuousPlanningParametersReadOnly, StoredPropertySetBasics
{
   default void setActiveMapping(boolean activeMapping)
   {
      set(ContinuousPlanningParameters.activeMapping, activeMapping);
   }

   default void setPauseContinuousWalking(boolean pauseContinuousWalking)
   {
      set(ContinuousPlanningParameters.pauseContinuousWalking, pauseContinuousWalking);
   }

   default void setNumberOfStepsToSend(int numberOfStepsToSend)
   {
      set(ContinuousPlanningParameters.numberOfStepsToSend, numberOfStepsToSend);
   }

   default void setMaxNumberOfStepsToHoldInControllerQueue(int maxNumberOfStepsToHoldInControllerQueue)
   {
      set(ContinuousPlanningParameters.maxNumberOfStepsToHoldInControllerQueue, maxNumberOfStepsToHoldInControllerQueue);
   }

   default void setGoalPoseForwardDistance(double goalPoseForwardDistance)
   {
      set(ContinuousPlanningParameters.goalPoseForwardDistance, goalPoseForwardDistance);
   }

   default void setGoalPoseUpDistance(double goalPoseUpDistance)
   {
      set(ContinuousPlanningParameters.goalPoseUpDistance, goalPoseUpDistance);
   }

   default void setSwingTime(double swingTime)
   {
      set(ContinuousPlanningParameters.swingTime, swingTime);
   }

   default void setTransferTime(double transferTime)
   {
      set(ContinuousPlanningParameters.transferTime, transferTime);
   }
}

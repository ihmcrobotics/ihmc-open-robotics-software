package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.activeMapping.ContinuousPlanningParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousPlanningParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getActiveMapping()
   {
      return get(activeMapping);
   }

   default boolean getPauseContinuousWalking()
   {
      return get(pauseContinuousWalking);
   }

   default int getNumberOfStepsToSend()
   {
      return get(numberOfStepsToSend);
   }

   default int getMaxNumberOfStepsToHoldInControllerQueue()
   {
      return get(maxNumberOfStepsToHoldInControllerQueue);
   }

   default double getGoalPoseForwardDistance()
   {
      return get(goalPoseForwardDistance);
   }

   default double getGoalPoseUpDistance()
   {
      return get(goalPoseUpDistance);
   }

   default double getSwingTime()
   {
      return get(swingTime);
   }

   default double getTransferTime()
   {
      return get(transferTime);
   }
}

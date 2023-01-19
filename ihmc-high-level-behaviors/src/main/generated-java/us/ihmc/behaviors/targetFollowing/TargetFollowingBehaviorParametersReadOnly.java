package us.ihmc.behaviors.targetFollowing;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface TargetFollowingBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getMinimumDistanceToKeepFromTarget()
   {
      return get(minimumDistanceToKeepFromTarget);
   }

   default double getLookAndStepGoalUpdatePeriod()
   {
      return get(lookAndStepGoalUpdatePeriod);
   }

   default double getTestLoopRadius()
   {
      return get(testLoopRadius);
   }
}

package us.ihmc.behaviors.targetFollowing;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface TargetFollowingBehaviorParametersBasics extends TargetFollowingBehaviorParametersReadOnly, StoredPropertySetBasics
{
   default void setMinimumDistanceToKeepFromTarget(double minimumDistanceToKeepFromTarget)
   {
      set(TargetFollowingBehaviorParameters.minimumDistanceToKeepFromTarget, minimumDistanceToKeepFromTarget);
   }

   default void setLookAndStepGoalUpdatePeriod(double lookAndStepGoalUpdatePeriod)
   {
      set(TargetFollowingBehaviorParameters.lookAndStepGoalUpdatePeriod, lookAndStepGoalUpdatePeriod);
   }

   default void setTestLoopRadius(double testLoopRadius)
   {
      set(TargetFollowingBehaviorParameters.testLoopRadius, testLoopRadius);
   }
}

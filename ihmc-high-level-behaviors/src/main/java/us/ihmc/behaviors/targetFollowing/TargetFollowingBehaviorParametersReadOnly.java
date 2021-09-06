package us.ihmc.behaviors.targetFollowing;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorParameters.*;

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

package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;

public interface NaturalPostureParameters
{
   String[] getJointsWithRestrictiveLimits();

   JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);
}

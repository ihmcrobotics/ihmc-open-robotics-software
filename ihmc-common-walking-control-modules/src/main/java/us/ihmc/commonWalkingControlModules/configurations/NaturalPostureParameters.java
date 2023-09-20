package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class NaturalPostureParameters
{
   public abstract String[] getJointsWithRestrictiveLimits();

   public abstract JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);

   private final Vector3D npQPWeights = new Vector3D(0.01, 0.01, 1);

   public Vector3D getQPWeights()
   {
      return npQPWeights;
   }

}

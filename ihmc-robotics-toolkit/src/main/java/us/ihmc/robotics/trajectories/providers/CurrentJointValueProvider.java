package us.ihmc.robotics.trajectories.providers;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class CurrentJointValueProvider implements DoubleProvider
{
   OneDoFJoint joint;
   
   public CurrentJointValueProvider(OneDoFJoint joint)
   {
      this.joint = joint;
   }
   
   public double getValue()
   {
      return joint.getQ();
   }

}

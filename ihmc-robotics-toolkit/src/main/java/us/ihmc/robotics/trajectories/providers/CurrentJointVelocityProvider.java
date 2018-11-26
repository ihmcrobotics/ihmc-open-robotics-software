package us.ihmc.robotics.trajectories.providers;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class CurrentJointVelocityProvider implements DoubleProvider
{
   OneDoFJointBasics joint;
   
   public CurrentJointVelocityProvider(OneDoFJointBasics joint)
   {
      this.joint = joint;
   }
   
   public double getValue()
   {
      return joint.getQd();
   }
}

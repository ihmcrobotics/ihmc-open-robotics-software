package us.ihmc.robotics.trajectories.providers;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class CurrentJointValueProvider implements DoubleProvider
{
   OneDoFJointBasics joint;
   
   public CurrentJointValueProvider(OneDoFJointBasics joint)
   {
      this.joint = joint;
   }
   
   public double getValue()
   {
      return joint.getQ();
   }

}

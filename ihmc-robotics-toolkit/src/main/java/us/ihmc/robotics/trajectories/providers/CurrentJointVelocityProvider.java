package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class CurrentJointVelocityProvider implements DoubleProvider
{
   OneDoFJoint joint;
   
   public CurrentJointVelocityProvider(OneDoFJoint joint)
   {
      this.joint = joint;
   }
   
   public double getValue()
   {
      return joint.getQd();
   }
}

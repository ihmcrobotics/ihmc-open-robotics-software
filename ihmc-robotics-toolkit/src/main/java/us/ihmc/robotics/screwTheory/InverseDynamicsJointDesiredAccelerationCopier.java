package us.ihmc.robotics.screwTheory;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class InverseDynamicsJointDesiredAccelerationCopier extends AbstractInverseDynamicsCopier
{

   public InverseDynamicsJointDesiredAccelerationCopier(RigidBodyBasics originalBody, RigidBodyBasics targetBody)
   {
      super(originalBody, targetBody);
   }

   @Override
   protected void copyJoint(JointBasics originalJoint, JointBasics targetJoint)
   {
      targetJoint.setJointAcceleration(originalJoint);

   }  
  
   
}

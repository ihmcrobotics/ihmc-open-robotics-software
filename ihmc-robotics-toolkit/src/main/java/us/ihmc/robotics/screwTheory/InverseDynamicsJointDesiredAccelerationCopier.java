package us.ihmc.robotics.screwTheory;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public class InverseDynamicsJointDesiredAccelerationCopier extends AbstractInverseDynamicsCopier
{

   public InverseDynamicsJointDesiredAccelerationCopier(RigidBody originalBody, RigidBody targetBody)
   {
      super(originalBody, targetBody);
   }

   @Override
   protected void copyJoint(JointBasics originalJoint, JointBasics targetJoint)
   {
      targetJoint.setQddDesired(originalJoint);

   }  
  
   
}

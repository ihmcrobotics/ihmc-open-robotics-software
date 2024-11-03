package us.ihmc.robotics.screwTheory;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class InverseDynamicsJointStateCopier extends AbstractInverseDynamicsCopier
{

   public InverseDynamicsJointStateCopier(RigidBodyBasics originalBody, RigidBodyBasics targetBody)
   {
      super(originalBody, targetBody);
   }

   @Override
   protected void copyJoint(JointBasics originalJoint, JointBasics targetJoint)
   {
      targetJoint.setJointConfiguration(originalJoint);
      targetJoint.setJointTwist(originalJoint);
      targetJoint.setJointAcceleration(originalJoint);
   }  

}

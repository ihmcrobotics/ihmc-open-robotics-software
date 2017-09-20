package us.ihmc.robotics.screwTheory;


public class InverseDynamicsJointStateCopier extends AbstractInverseDynamicsCopier
{

   public InverseDynamicsJointStateCopier(RigidBody originalBody, RigidBody targetBody)
   {
      super(originalBody, targetBody);
   }

   @Override
   protected void copyJoint(InverseDynamicsJoint originalJoint, InverseDynamicsJoint targetJoint)
   {
      targetJoint.setJointPositionVelocityAndAcceleration(originalJoint);
   }  

}

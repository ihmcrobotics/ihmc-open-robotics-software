package us.ihmc.robotics.screwTheory;


public class InverseDynamicsJointDesiredAccelerationCopier extends AbstractInverseDynamicsCopier
{

   public InverseDynamicsJointDesiredAccelerationCopier(RigidBody originalBody, RigidBody targetBody)
   {
      super(originalBody, targetBody);
   }

   @Override
   protected void copyJoint(InverseDynamicsJoint originalJoint, InverseDynamicsJoint targetJoint)
   {
      targetJoint.setQddDesired(originalJoint);

   }  
  
   
}

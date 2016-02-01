package us.ihmc.robotics.screwTheory;

public class InverseDynamicsJointDesiredAccelerationChecksum extends AbstractInverseDynamicsChecksum
{

   public InverseDynamicsJointDesiredAccelerationChecksum(RigidBody rootJoint, GenericCRC32 checksum)
   {
      super(rootJoint, checksum);
   }

   @Override
   public void calculateJointChecksum(InverseDynamicsJoint joint)
   {
      joint.calculateJointDesiredChecksum(checksum);
   }

}

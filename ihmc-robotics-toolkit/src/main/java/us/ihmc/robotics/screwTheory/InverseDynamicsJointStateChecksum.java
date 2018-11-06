package us.ihmc.robotics.screwTheory;

public class InverseDynamicsJointStateChecksum extends AbstractInverseDynamicsChecksum
{

   public InverseDynamicsJointStateChecksum(RigidBody rootJoint, GenericCRC32 checksum)
   {
      super(rootJoint, checksum);
   }

   @Override
   public void calculateJointChecksum(JointBasics joint)
   {
      joint.calculateJointStateChecksum(checksum);
   }

}

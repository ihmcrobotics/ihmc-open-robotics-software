package us.ihmc.robotics.screwTheory;

import us.ihmc.tools.compare.GenericCRC32;

public abstract class AbstractInverseDynamicsChecksum
{
   private final InverseDynamicsJoint[] joints;
   protected final GenericCRC32 checksum;
   
   public AbstractInverseDynamicsChecksum(RigidBody rootJoint, GenericCRC32 checksum)
   {
      this.checksum = checksum;
      this.joints = ScrewTools.computeSubtreeJoints(rootJoint);
   }
   
   public void calculate()
   {
      for(InverseDynamicsJoint joint : joints)
      {
         calculateJointChecksum(joint);
      }
   }
   
   public abstract void calculateJointChecksum(InverseDynamicsJoint joint); 
   
}

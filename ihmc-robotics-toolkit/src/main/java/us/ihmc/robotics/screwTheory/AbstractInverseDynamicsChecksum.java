package us.ihmc.robotics.screwTheory;

import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public abstract class AbstractInverseDynamicsChecksum
{
   private final JointBasics[] joints;
   protected final GenericCRC32 checksum;
   
   public AbstractInverseDynamicsChecksum(RigidBody rootJoint, GenericCRC32 checksum)
   {
      this.checksum = checksum;
      this.joints = ScrewTools.computeSubtreeJoints(rootJoint);
   }
   
   public void calculate()
   {
      for(JointBasics joint : joints)
      {
         calculateJointChecksum(joint);
      }
   }
   
   public abstract void calculateJointChecksum(JointBasics joint); 
   
}

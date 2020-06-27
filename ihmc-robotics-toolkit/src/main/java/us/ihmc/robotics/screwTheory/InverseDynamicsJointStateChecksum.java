package us.ihmc.robotics.screwTheory;

import us.ihmc.mecano.multiBodySystem.PlanarJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.SphericalJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;

public class InverseDynamicsJointStateChecksum
{
   protected final GenericCRC32 checksum;
   private final ChecksumUpdater[] checksumUpdaters;

   public InverseDynamicsJointStateChecksum(RigidBodyBasics rootBody, GenericCRC32 checksum)
   {
      this.checksum = checksum;

      checksumUpdaters = SubtreeStreams.fromChildren(rootBody).map(joint -> newJointChecksumUpdater(checksum, joint)).toArray(ChecksumUpdater[]::new);
   }

   public void calculate()
   {
      for (ChecksumUpdater checksumUpdater : checksumUpdaters)
      {
         checksumUpdater.updateChecksum();
      }
   }

   public static ChecksumUpdater newJointChecksumUpdater(GenericCRC32 checksum, JointBasics joint)
   {
      if (joint instanceof SixDoFJoint)
         return newJointChecksumUpdater(checksum, (SixDoFJoint) joint);
      if (joint instanceof PlanarJoint)
         return newJointChecksumUpdater(checksum, (PlanarJoint) joint);
      if (joint instanceof OneDoFJointBasics)
         return newJointChecksumUpdater(checksum, (OneDoFJointBasics) joint);
      if (joint instanceof SphericalJoint)
         return newJointChecksumUpdater(checksum, (SphericalJoint) joint);
      throw new RuntimeException("Unhandled type of joint: " + joint.getClass().getSimpleName());
   }

   public static ChecksumUpdater newJointChecksumUpdater(GenericCRC32 checksum, SixDoFJoint joint)
   {
      return () -> {
         checksum.update(joint.getJointPose().getOrientation());
         checksum.update(joint.getJointPose().getPosition());
         checksum.update(joint.getJointTwist().getAngularPart());
         checksum.update(joint.getJointTwist().getLinearPart());
         checksum.update(joint.getJointAcceleration().getAngularPart());
         checksum.update(joint.getJointAcceleration().getLinearPart());
      };
   }

   public static ChecksumUpdater newJointChecksumUpdater(GenericCRC32 checksum, PlanarJoint joint)
   {
      return () -> {
         checksum.update(joint.getJointPose().getOrientation().getPitch());
         checksum.update(joint.getJointPose().getPosition().getX());
         checksum.update(joint.getJointPose().getPosition().getY());
         checksum.update(joint.getJointTwist().getAngularPartY());
         checksum.update(joint.getJointTwist().getLinearPartX());
         checksum.update(joint.getJointTwist().getLinearPartZ());
         checksum.update(joint.getJointAcceleration().getAngularPartY());
         checksum.update(joint.getJointAcceleration().getLinearPartX());
         checksum.update(joint.getJointAcceleration().getLinearPartZ());
      };
   }

   public static ChecksumUpdater newJointChecksumUpdater(GenericCRC32 checksum, OneDoFJointBasics joint)
   {
      return () -> {
         checksum.update(joint.getQ());
         checksum.update(joint.getQd());
         checksum.update(joint.getQdd());
      };
   }

   public static ChecksumUpdater newJointChecksumUpdater(GenericCRC32 checksum, SphericalJoint joint)
   {
      return () -> {
         checksum.update(joint.getJointOrientation());
         checksum.update(joint.getJointAngularVelocity());
         checksum.update(joint.getJointAngularAcceleration());
      };
   }

   public static interface ChecksumUpdater
   {
      void updateChecksum();
   }
}

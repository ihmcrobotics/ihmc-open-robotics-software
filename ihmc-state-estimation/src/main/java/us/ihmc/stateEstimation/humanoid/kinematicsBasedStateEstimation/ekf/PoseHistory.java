package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import us.ihmc.commons.lists.RingBuffer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import javax.annotation.Nullable;

/** Bounded exposure pairing. No extrapolation, no corrected-state substitution. */
public final class PoseHistory
{
   private static final int CAPACITY = 300;
   private final RingBuffer<TimeStampedPose> poses = new RingBuffer<>(CAPACITY, TimeStampedPose::new);
   private final Pose3D interpolatedPose = new Pose3D();

   public PoseHistory()
   {
   }

   public void clear()
   {
      poses.reset();
   }

   public void add(long wall, RigidBodyTransformReadOnly pose)
   {
      if (wall <= 0 || pose.containsNaN() || (!poses.isEmpty() && wall <= poses.getLast().getTimestamp()))
         return;
      poses.add().set(pose, wall);
   }

   /**
    * Returns the pose at the specified timestamp. Linearly interpolates between stamped poses when timestamp is between poses.
    * @return false if {@param timestamp} is invalid.
    **/
   public boolean getPoseAtTime(long timestamp, @Nullable RigidBodyTransformBasics poseToPack)
   {
      int previous = -1;
      for (int index = 0; index < poses.size(); index++)
      {
         TimeStampedPose stampedPose = poses.getFromFirst(index);
         if (stampedPose.getTimestamp() == timestamp)
         {
            if (poseToPack != null)
               poseToPack.set(stampedPose.getPose());
            return true;
         }
         if (stampedPose.getTimestamp() > timestamp)
         {
            if (previous < 0)
               return false;
            TimeStampedPose previousStampedPose = poses.getFromFirst(previous);
            if (stampedPose.getTimestamp() - previousStampedPose.getTimestamp() > 50_000_000L)
               return false;

            double alpha = (double) (timestamp - previousStampedPose.getTimestamp()) / (stampedPose.getTimestamp() - previousStampedPose.getTimestamp());
            interpolatedPose.interpolate(previousStampedPose.getPose(), stampedPose.getPose(), alpha);
            if (poseToPack != null)
               poseToPack.set(interpolatedPose);
            return true;
         }
         previous = index;
      }
      return false;
   }

   private static class TimeStampedPose
   {
      private final Pose3D pose = new Pose3D();
      private long timestamp;

      public void set(RigidBodyTransformReadOnly pose, long timestamp)
      {
         this.pose.set(pose);
         this.timestamp = timestamp;
      }

      public Pose3DReadOnly getPose()
      {
         return pose;
      }

      public long getTimestamp()
      {
         return timestamp;
      }
   }
}

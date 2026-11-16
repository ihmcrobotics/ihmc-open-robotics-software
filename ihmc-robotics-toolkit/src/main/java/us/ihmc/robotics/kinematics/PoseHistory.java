package us.ihmc.robotics.kinematics;

import us.ihmc.commons.lists.RingBuffer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import javax.annotation.Nullable;

/**
 * A bounded history of timestamped poses, for looking up a pose at a past time, such as a measurement's exposure time.
 * <p>
 * Timestamps must increase: a pose that isn't newer than the newest one is ignored. A lookup between two poses interpolates
 * between them, if they're at most {@link #MAXIMUM_INTERPOLATION_GAP_NANOSECONDS} apart. Nothing is extrapolated. Allocation-free
 * after construction.
 * </p>
 */
public final class PoseHistory
{
   /** Two poses further apart than this, in ns, aren't interpolated between. */
   public static final long MAXIMUM_INTERPOLATION_GAP_NANOSECONDS = 50_000_000L;
   public static final int DEFAULT_CAPACITY = 300;
   private final RingBuffer<TimeStampedPose> poses;
   private final Pose3D interpolatedPose = new Pose3D();

   public PoseHistory()
   {
      this(DEFAULT_CAPACITY);
   }

   /** @param capacity how many poses are kept; adding one more drops the oldest. */
   public PoseHistory(int capacity)
   {
      poses = new RingBuffer<>(capacity, TimeStampedPose::new);
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
            if (stampedPose.getTimestamp() - previousStampedPose.getTimestamp() > MAXIMUM_INTERPOLATION_GAP_NANOSECONDS)
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

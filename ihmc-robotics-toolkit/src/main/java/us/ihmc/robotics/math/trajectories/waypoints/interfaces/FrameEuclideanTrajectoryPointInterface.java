package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.interfaces.FrameEuclideanWaypointInterface;

public interface FrameEuclideanTrajectoryPointInterface extends EuclideanTrajectoryPointInterface, FrameEuclideanWaypointInterface
{
   public default void set(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   public default void setIncludingFrame(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      setIncludingFrame(position, linearVelocity);
   }

   default void set(FrameEuclideanTrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameEuclideanWaypointInterface.super.set(other);
   }

   default boolean epsilonEquals(FrameEuclideanTrajectoryPointInterface other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameEuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
   }
}

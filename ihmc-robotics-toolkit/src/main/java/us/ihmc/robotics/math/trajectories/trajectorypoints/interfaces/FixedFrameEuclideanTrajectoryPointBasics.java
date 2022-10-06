package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public interface FixedFrameEuclideanTrajectoryPointBasics
      extends FrameEuclideanTrajectoryPointReadOnly, EuclideanTrajectoryPointBasics, FixedFrameEuclideanWaypointBasics
{
   default void set(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(double time, FrameEuclideanWaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(FrameEuclideanTrajectoryPointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, EuclideanTrajectoryPointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }
}
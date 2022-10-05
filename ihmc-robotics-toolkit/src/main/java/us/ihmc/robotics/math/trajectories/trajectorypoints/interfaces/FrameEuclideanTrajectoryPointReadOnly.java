package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public interface FrameEuclideanTrajectoryPointReadOnly extends EuclideanTrajectoryPointReadOnly, FrameEuclideanWaypointReadOnly
{
   @Deprecated
   default void get(FixedFrameEuclideanTrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   @Deprecated
   default void getIncludingFrame(FrameEuclideanTrajectoryPointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   @Override
   default String toString(String format)
   {
      return EuclideanTrajectoryPointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
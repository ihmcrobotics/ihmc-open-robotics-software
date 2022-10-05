package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public interface FrameSO3TrajectoryPointReadOnly extends SO3TrajectoryPointReadOnly, FrameSO3WaypointReadOnly
{
   @Deprecated
   default void getIncludingFrame(FrameSO3TrajectoryPointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   @Deprecated
   default void get(FixedFrameSO3TrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   @Override
   default String toString(String format)
   {
      return SO3TrajectoryPointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
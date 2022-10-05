package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public interface FrameSO3TrajectoryPointReadOnly extends SO3TrajectoryPointReadOnly, FrameSO3WaypointReadOnly
{
   @Override
   default String toString(String format)
   {
      return SO3TrajectoryPointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
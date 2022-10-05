package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;

public interface FrameSE3TrajectoryPointReadOnly
      extends SE3TrajectoryPointReadOnly, FrameSE3WaypointReadOnly, FrameEuclideanTrajectoryPointReadOnly, FrameSO3TrajectoryPointReadOnly
{
   @Override
   default String toString(String format)
   {
      return SE3TrajectoryPointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }

}
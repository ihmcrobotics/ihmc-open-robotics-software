package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public interface FrameEuclideanTrajectoryPointReadOnly extends EuclideanTrajectoryPointReadOnly, FrameEuclideanWaypointReadOnly
{
   @Override
   default String toString(String format)
   {
      return EuclideanTrajectoryPointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
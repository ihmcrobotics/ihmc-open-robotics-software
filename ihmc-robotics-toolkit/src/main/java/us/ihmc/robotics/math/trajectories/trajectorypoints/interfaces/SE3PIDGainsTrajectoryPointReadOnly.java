package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3PIDGainsWaypointReadOnly;

public interface SE3PIDGainsTrajectoryPointReadOnly extends TrajectoryPointReadOnly, SE3PIDGainsWaypointReadOnly
{
   @Override
   default boolean containsNaN()
   {
      return isTimeNaN() || SE3PIDGainsWaypointReadOnly.super.containsNaN();
   }


   default boolean equals(SE3PIDGainsTrajectoryPointReadOnly other)
   {
      return EuclidCoreTools.equals(getTime(), other.getTime()) && SE3PIDGainsWaypointReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(SE3PIDGainsTrajectoryPointReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon) && SE3PIDGainsWaypointReadOnly.super.epsilonEquals(other, epsilon);
   }
}

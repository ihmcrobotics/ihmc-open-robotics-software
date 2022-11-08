package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFWaypointReadOnly;

public interface OneDoFTrajectoryPointReadOnly extends TrajectoryPointReadOnly, OneDoFWaypointReadOnly
{
   @Override
   default boolean containsNaN()
   {
      return isTimeNaN() || OneDoFWaypointReadOnly.super.containsNaN();
   }

   default boolean equals(OneDoFTrajectoryPointReadOnly other)
   {
      return EuclidCoreTools.equals(getTime(), other.getTime()) && OneDoFWaypointReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(OneDoFTrajectoryPointReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon) && OneDoFWaypointReadOnly.super.epsilonEquals(other, epsilon);
   }
}
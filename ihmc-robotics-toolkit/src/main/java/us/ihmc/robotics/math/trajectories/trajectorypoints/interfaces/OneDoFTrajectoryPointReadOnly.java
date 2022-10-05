package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFWaypointReadOnly;

public interface OneDoFTrajectoryPointReadOnly extends TrajectoryPointReadOnly, OneDoFWaypointReadOnly
{
   default boolean epsilonEquals(OneDoFTrajectoryPointBasics other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon) && OneDoFWaypointReadOnly.super.epsilonEquals(other, epsilon);
   }
}
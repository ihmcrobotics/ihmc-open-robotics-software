package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;

public interface EuclideanTrajectoryPointInterface<T extends EuclideanTrajectoryPointInterface<T>>
      extends TrajectoryPointInterface<T>, EuclideanWaypointInterface<T>
{
}
package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;

public interface SE3TrajectoryPointInterface<T extends SE3TrajectoryPointInterface<T>>
      extends EuclideanTrajectoryPointInterface<T>, SO3TrajectoryPointInterface<T>, SE3WaypointInterface<T>
{
}
package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface SE3TrajectoryPointInterface<T extends SE3TrajectoryPointInterface<T>>
      extends EuclideanTrajectoryPointInterface<T>, SO3TrajectoryPointInterface<T>, SE3WaypointInterface<T>
{
}
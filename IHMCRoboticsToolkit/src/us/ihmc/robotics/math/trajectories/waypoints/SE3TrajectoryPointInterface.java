package us.ihmc.robotics.math.trajectories.waypoints;

public interface SE3TrajectoryPointInterface<T extends SE3TrajectoryPointInterface<T>>
      extends EuclideanTrajectoryPointInterface<T>, SO3TrajectoryPointInterface<T>
{
}
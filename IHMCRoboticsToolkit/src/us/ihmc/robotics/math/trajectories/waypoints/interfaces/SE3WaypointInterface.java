package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface SE3WaypointInterface<T extends SE3WaypointInterface<T>> extends EuclideanWaypointInterface<T>, SO3WaypointInterface<T>
{
}

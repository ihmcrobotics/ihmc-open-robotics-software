package us.ihmc.robotics.math.trajectories.waypoints;

public interface TrajectoryWaypointDataInterface<T extends WaypointInterface>
{
   public abstract int getNumberOfWaypoints();

   public abstract T getWaypoint(int waypointIndex);

   public abstract T getLastWaypoint();

   public abstract double getTrajectoryTime();
}

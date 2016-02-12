package us.ihmc.robotics.math.trajectories;

public interface TrajectoryWaypoint1DDataInterface
{
   public abstract Waypoint1DInterface getWaypoint(int waypointIndex);

   public abstract Waypoint1DInterface[] getWaypoints();
}

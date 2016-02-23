package us.ihmc.robotics.math.trajectories;

public interface TrajectoryWaypoint1DDataInterface
{
   public abstract int getNumberOfWaypoints();

   public abstract Waypoint1DInterface getWaypoint(int waypointIndex);

   public abstract Waypoint1DInterface getLastWaypoint();

   public abstract double getTrajectoryTime();
}

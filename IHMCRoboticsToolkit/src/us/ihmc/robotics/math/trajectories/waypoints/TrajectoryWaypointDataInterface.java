package us.ihmc.robotics.math.trajectories.waypoints;

public interface TrajectoryWaypointDataInterface<T extends TrajectoryWaypointDataInterface<T, W>, W extends WaypointInterface<W>>
{
   public abstract void clear();

   public abstract void addWaypoint(W waypoint);

   public abstract void set(T trajectory);

   public abstract int getNumberOfWaypoints();

   public abstract W getWaypoint(int waypointIndex);

   public abstract W getLastWaypoint();

   public abstract double getTrajectoryTime();

   public abstract boolean epsilonEquals(T other, double epsilon);
}

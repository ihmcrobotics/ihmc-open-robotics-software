package us.ihmc.robotics.math.trajectories.waypoints;

public interface Waypoint1DInterface<T extends Waypoint1DInterface<T>> extends WaypointInterface<T>
{
   public abstract double getPosition();
   public abstract double getVelocity();
}

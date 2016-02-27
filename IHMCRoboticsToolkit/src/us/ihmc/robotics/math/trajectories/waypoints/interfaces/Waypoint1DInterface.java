package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface Waypoint1DInterface<T extends Waypoint1DInterface<T>> extends WaypointInterface<T>
{
   public abstract void setPosition(double position);

   public abstract void setVelocity(double velocity);

   public abstract double getPosition();

   public abstract double getVelocity();
}

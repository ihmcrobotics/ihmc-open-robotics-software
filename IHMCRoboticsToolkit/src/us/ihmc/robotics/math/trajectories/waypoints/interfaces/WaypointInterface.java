package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface WaypointInterface<T extends WaypointInterface<T>>
{
   public abstract void set(T other);

   public abstract void setToZero();

   public abstract void setToNaN();

   public abstract boolean containsNaN();

   public abstract boolean epsilonEquals(T other, double epsilon);
}

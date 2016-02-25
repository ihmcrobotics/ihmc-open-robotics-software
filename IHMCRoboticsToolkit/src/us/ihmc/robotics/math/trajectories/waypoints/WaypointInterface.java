package us.ihmc.robotics.math.trajectories.waypoints;

public interface WaypointInterface<T extends WaypointInterface<T>>
{
   public abstract void setTime(double time);
   public abstract double getTime();
   public abstract void set(T waypoint);
   public abstract void addTimeOffset(double timeOffsetToAdd);
   public abstract void subtractTimeOffset(double timeOffsetToSubtract);
   public abstract boolean epsilonEquals(T other, double epsilon);
}

package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface TrajectoryPointInterface<T extends TrajectoryPointInterface<T>> extends WaypointInterface<T>
{
   public abstract void setTime(double time);

   public abstract void addTimeOffset(double timeOffsetToAdd);

   public abstract void subtractTimeOffset(double timeOffsetToSubtract);

   public abstract void setTimeToZero();

   public abstract void setTimeToNaN();

   public abstract double getTime();
}

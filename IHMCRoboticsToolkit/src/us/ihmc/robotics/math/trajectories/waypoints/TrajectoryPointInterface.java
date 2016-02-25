package us.ihmc.robotics.math.trajectories.waypoints;

public interface TrajectoryPointInterface<T extends TrajectoryPointInterface<T>>
{
   public abstract void setTime(double time);

   public abstract double getTime();

   public abstract void set(T other);

   public abstract void addTimeOffset(double timeOffsetToAdd);

   public abstract void subtractTimeOffset(double timeOffsetToSubtract);

   public abstract boolean epsilonEquals(T other, double epsilon);
}

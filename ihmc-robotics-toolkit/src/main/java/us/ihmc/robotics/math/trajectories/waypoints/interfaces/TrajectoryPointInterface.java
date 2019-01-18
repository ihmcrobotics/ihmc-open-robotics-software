package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface TrajectoryPointInterface
{
   public abstract void setTime(double time);

   public abstract double getTime();

   public default void addTimeOffset(double timeOffsetToAdd)
   {
      setTime(getTime() + timeOffsetToAdd);
   }

   public default void subtractTimeOffset(double timeOffsetToSubtract)
   {
      setTime(getTime() - timeOffsetToSubtract);
   }

   public default void setTimeToZero()
   {
      setTime(0.0);
   }

   public default void setTimeToNaN()
   {
      setTime(Double.NaN);
   }
}

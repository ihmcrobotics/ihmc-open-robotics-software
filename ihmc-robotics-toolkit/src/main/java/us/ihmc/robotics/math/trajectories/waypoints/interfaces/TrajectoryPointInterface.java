package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface TrajectoryPointInterface
{
   abstract void setTime(double time);

   abstract double getTime();

   default void addTimeOffset(double timeOffsetToAdd)
   {
      setTime(getTime() + timeOffsetToAdd);
   }

   default void subtractTimeOffset(double timeOffsetToSubtract)
   {
      setTime(getTime() - timeOffsetToSubtract);
   }

   default void setTimeToZero()
   {
      setTime(0.0);
   }

   default void setTimeToNaN()
   {
      setTime(Double.NaN);
   }
}

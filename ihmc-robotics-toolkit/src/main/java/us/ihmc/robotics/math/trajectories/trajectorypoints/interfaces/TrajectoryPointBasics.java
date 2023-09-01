package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

public interface TrajectoryPointBasics extends TrajectoryPointReadOnly
{
   void setTime(double time);

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

package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

public interface TrajectoryPointReadOnly
{
   double getTime();

   default boolean isTimeNaN()
   {
      return Double.isNaN(getTime());
   }

   default boolean isTimeZero()
   {
      return getTime() == 0.0;
   }
}
package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFWaypointBasics;

public interface OneDoFTrajectoryPointBasics extends OneDoFTrajectoryPointReadOnly, TrajectoryPointBasics, OneDoFWaypointBasics
{
   default void set(double time, double position, double velocity, double acceleration)
   {
      setTime(time);
      set(position, velocity, acceleration);
   }

   default void set(OneDoFTrajectoryPointBasics other)
   {
      setTime(other.getTime());
      OneDoFWaypointBasics.super.set(other);
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      OneDoFWaypointBasics.super.setToZero();
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      OneDoFWaypointBasics.super.setToNaN();
   }

   @Override
   default boolean containsNaN()
   {
      return OneDoFTrajectoryPointReadOnly.super.containsNaN();
   }
}

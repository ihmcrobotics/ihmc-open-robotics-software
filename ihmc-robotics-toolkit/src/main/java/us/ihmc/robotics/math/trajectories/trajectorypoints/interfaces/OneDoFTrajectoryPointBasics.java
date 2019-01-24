package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFWaypointBasics;

public interface OneDoFTrajectoryPointBasics extends TrajectoryPointBasics, OneDoFWaypointBasics
{
   default void set(double time, double position, double velocity)
   {
      setTime(time);
      set(position, velocity);
   }

   default void set(OneDoFTrajectoryPointBasics other)
   {
      setTime(other.getTime());
      OneDoFWaypointBasics.super.set(other);
   }

   default boolean epsilonEquals(OneDoFTrajectoryPointBasics other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon) && OneDoFWaypointBasics.super.epsilonEquals(other, epsilon);
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
      return Double.isNaN(getTime()) || OneDoFWaypointBasics.super.containsNaN();
   }
}

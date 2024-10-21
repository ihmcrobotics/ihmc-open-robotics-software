package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3PIDGainsWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3PIDGainsWaypointReadOnly;

public interface SE3PIDGainsTrajectoryPointBasics extends SE3PIDGainsTrajectoryPointReadOnly, TrajectoryPointBasics, SE3PIDGainsWaypointBasics
{
   default void set(double time, PID3DGains angular, PID3DGains linear)
   {
      setTime(time);
      set(angular, linear);
   }

   default void set(SE3PIDGainsTrajectoryPointBasics other)
   {
      setTime(other.getTime());
      SE3PIDGainsWaypointBasics.super.set(other);
   }

   default void set(double time, SE3PIDGainsWaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SE3PIDGainsWaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SE3PIDGainsWaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return SE3PIDGainsTrajectoryPointReadOnly.super.containsNaN();
   }
}

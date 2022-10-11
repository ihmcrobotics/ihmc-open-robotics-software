package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;

public interface SO3TrajectoryPointBasics extends SO3TrajectoryPointReadOnly, TrajectoryPointBasics, SO3WaypointBasics
{
   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SO3WaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SO3WaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return SO3TrajectoryPointReadOnly.super.containsNaN();
   }

   default void set(double time, Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   default void set(SO3TrajectoryPointReadOnly other)
   {
      set(other.getTime(), other);
   }

   default void set(double time, SO3WaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }
}

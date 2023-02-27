package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;

public interface SE3TrajectoryPointBasics
      extends SE3TrajectoryPointReadOnly, TrajectoryPointBasics, SE3WaypointBasics, EuclideanTrajectoryPointBasics, SO3TrajectoryPointBasics
{
   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SE3WaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SE3WaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return SE3TrajectoryPointReadOnly.super.containsNaN();
   }

   default void set(double time, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   default void set(SE3TrajectoryPointReadOnly other)
   {
      set(other.getTime(), other);
   }

   default void set(double time, SE3WaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(double time, EuclideanWaypointReadOnly euclideanWaypoint, SO3WaypointReadOnly so3Waypoint)
   {
      setTime(time);
      set(euclideanWaypoint);
      set(so3Waypoint);
   }
}
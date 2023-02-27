package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;

public interface EuclideanTrajectoryPointBasics extends EuclideanTrajectoryPointReadOnly, TrajectoryPointBasics, EuclideanWaypointBasics
{
   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      EuclideanWaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      EuclideanWaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanTrajectoryPointReadOnly.super.containsNaN();
   }

   default void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(EuclideanTrajectoryPointReadOnly other)
   {
      set(other.getTime(), other);
   }

   default void set(double time, EuclideanWaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }
}
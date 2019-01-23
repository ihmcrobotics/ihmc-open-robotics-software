package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;

public interface SO3TrajectoryPointBasics extends TrajectoryPointBasics, SO3WaypointBasics
{
   default void set(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   default void set(SO3TrajectoryPointBasics other)
   {
      setTime(other.getTime());
      SO3WaypointBasics.super.set(other);
   }

   default void set(double time, SO3WaypointBasics waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void get(SO3TrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(SO3TrajectoryPointBasics other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && SO3WaypointBasics.super.epsilonEquals(other, epsilon);
   }

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
      return Double.isNaN(getTime()) || SO3WaypointBasics.super.containsNaN();
   }
}

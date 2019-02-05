package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;

public interface SE3TrajectoryPointBasics extends TrajectoryPointBasics, SE3WaypointBasics, EuclideanTrajectoryPointBasics, SO3TrajectoryPointBasics
{
   default void set(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   default void set(SE3TrajectoryPointBasics other)
   {
      setTime(other.getTime());
      SE3WaypointBasics.super.set(other);
   }

   default void set(double time, SE3WaypointBasics waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(double time, EuclideanWaypointBasics euclideanWaypoint, SO3WaypointBasics so3Waypoint)
   {
      setTime(time);
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void get(SE3TrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void get(EuclideanTrajectoryPointBasics euclideanTrajectoryPointToPack, SO3TrajectoryPointBasics so3TrajectoryPointToPack)
   {
      get(euclideanTrajectoryPointToPack);
      get(so3TrajectoryPointToPack);
   }

   default boolean epsilonEquals(SE3TrajectoryPointBasics other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && SE3WaypointBasics.super.epsilonEquals(other, epsilon);
   }

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
      return Double.isNaN(getTime()) || SE3WaypointBasics.super.containsNaN();
   }
}
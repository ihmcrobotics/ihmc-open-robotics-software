package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SE3WaypointBasics extends SE3WaypointReadOnly, EuclideanWaypointBasics, SO3WaypointBasics
{
   default void set(Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   default void set(SE3WaypointReadOnly other)
   {
      EuclideanWaypointBasics.super.set(other);
      SO3WaypointBasics.super.set(other);
   }

   @Override
   default void setToNaN()
   {
      EuclideanWaypointBasics.super.setToNaN();
      SO3WaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      EuclideanWaypointBasics.super.setToZero();
      SO3WaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return SE3WaypointReadOnly.super.containsNaN();
   }
}

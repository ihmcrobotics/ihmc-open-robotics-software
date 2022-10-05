package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;

public interface SE3WaypointBasics extends SE3WaypointReadOnly, EuclideanWaypointBasics, SO3WaypointBasics
{
   @Override
   EuclideanWaypointBasics getEuclideanWaypoint();

   @Override
   SO3WaypointBasics getSO3Waypoint();

   @Override
   default Point3DBasics getPosition()
   {
      return getEuclideanWaypoint().getPosition();
   }

   @Override
   default QuaternionBasics getOrientation()
   {
      return getSO3Waypoint().getOrientation();
   }

   @Override
   default Vector3DBasics getLinearVelocity()
   {
      return getEuclideanWaypoint().getLinearVelocity();
   }

   @Override
   default Vector3DBasics getAngularVelocity()
   {
      return getSO3Waypoint().getAngularVelocity();
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

   default void set(Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      getEuclideanWaypoint().set(position, linearVelocity);
      getSO3Waypoint().set(orientation, angularVelocity);
   }

   default void set(SE3WaypointReadOnly other)
   {
      EuclideanWaypointBasics.super.set(other);
      SO3WaypointBasics.super.set(other);
   }

   @Override
   default void applyTransform(Transform transform)
   {
      getEuclideanWaypoint().applyTransform(transform);
      getSO3Waypoint().applyTransform(transform);
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      getEuclideanWaypoint().applyInverseTransform(transform);
      getSO3Waypoint().applyInverseTransform(transform);
   }
}

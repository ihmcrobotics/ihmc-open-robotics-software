package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SE3WaypointReadOnly extends EuclideanWaypointReadOnly, SO3WaypointReadOnly
{
   EuclideanWaypointReadOnly getEuclideanWaypoint();

   SO3WaypointReadOnly getSO3Waypoint();

   @Override
   default Point3DReadOnly getPosition()
   {
      return getEuclideanWaypoint().getPosition();
   }

   @Override
   default QuaternionReadOnly getOrientation()
   {
      return getSO3Waypoint().getOrientation();
   }

   @Override
   default Vector3DReadOnly getLinearVelocity()
   {
      return getEuclideanWaypoint().getLinearVelocity();
   }

   @Override
   default Vector3DReadOnly getAngularVelocity()
   {
      return getSO3Waypoint().getAngularVelocity();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointReadOnly.super.containsNaN() || SO3WaypointReadOnly.super.containsNaN();
   }

   default void getPose(Pose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3WaypointReadOnly))
         return false;

      SE3WaypointReadOnly other = (SE3WaypointReadOnly) geometry;

      if (!getEuclideanWaypoint().equals(other.getEuclideanWaypoint()))
         return false;
      if (!getSO3Waypoint().equals(other.getSO3Waypoint()))
         return false;
      return true;
   }

   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3WaypointReadOnly))
         return false;

      SE3WaypointReadOnly other = (SE3WaypointReadOnly) geometry;

      if (!getEuclideanWaypoint().epsilonEquals(other.getEuclideanWaypoint(), epsilon))
         return false;
      if (!getSO3Waypoint().epsilonEquals(other.getSO3Waypoint(), epsilon))
         return false;
      return true;
   }

   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3WaypointReadOnly))
         return false;

      SE3WaypointReadOnly other = (SE3WaypointReadOnly) geometry;

      if (!getEuclideanWaypoint().geometricallyEquals(other.getEuclideanWaypoint(), epsilon))
         return false;
      if (!getSO3Waypoint().geometricallyEquals(other.getSO3Waypoint(), epsilon))
         return false;
      return true;
   }

   @Override
   default String toString(String format)
   {
      return String.format("SE3 waypoint: [position=%s, orientation=%s, linear velocity=%s, angular velocity=%s]",
                           getPosition().toString(format),
                           getOrientation().toString(format),
                           getLinearVelocity().toString(format),
                           getAngularVelocity().toString(format));
   }
}
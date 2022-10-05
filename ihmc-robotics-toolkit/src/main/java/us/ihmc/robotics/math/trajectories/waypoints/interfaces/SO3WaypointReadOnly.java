package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SO3WaypointReadOnly extends EuclidGeometry
{
   QuaternionReadOnly getOrientation();

   Vector3DReadOnly getAngularVelocity();

   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getAngularVelocity().containsNaN();
   }

   @Deprecated
   default QuaternionBasics getOrientationCopy()
   {
      return new Quaternion(getOrientation());
   }

   @Deprecated
   default Vector3DBasics getAngularVelocityCopy()
   {
      return new Vector3D(getAngularVelocity());
   }

   default double getOrientationX()
   {
      return getOrientation().getX();
   }

   default double getOrientationY()
   {
      return getOrientation().getY();
   }

   default double getOrientationZ()
   {
      return getOrientation().getZ();
   }

   default double getOrientationS()
   {
      return getOrientation().getS();
   }

   default double getAngularVelocityX()
   {
      return getAngularVelocity().getX();
   }

   default double getAngularVelocityY()
   {
      return getAngularVelocity().getY();
   }

   default double getAngularVelocityZ()
   {
      return getAngularVelocity().getZ();
   }

   default double orientationDistance(SO3WaypointReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   @Deprecated
   default void getOrientation(Orientation3DBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   @Deprecated
   default void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(getAngularVelocity());
   }

   @Deprecated
   default void get(SO3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void get(Orientation3DBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SO3WaypointReadOnly))
         return false;

      SO3WaypointReadOnly other = (SO3WaypointReadOnly) geometry;

      if (!getOrientation().equals(other.getOrientation()))
         return false;
      if (!getAngularVelocity().equals(other.getAngularVelocity()))
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
      if (!(geometry instanceof SO3WaypointReadOnly))
         return false;

      SO3WaypointReadOnly other = (SO3WaypointReadOnly) geometry;

      if (!getOrientation().epsilonEquals(other.getOrientation(), epsilon))
         return false;
      if (!getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon))
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
      if (!(geometry instanceof SO3WaypointReadOnly))
         return false;

      SO3WaypointReadOnly other = (SO3WaypointReadOnly) geometry;

      if (!getOrientation().geometricallyEquals(other.getOrientation(), epsilon))
         return false;
      if (!getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   default String toString(String format)
   {
      return String.format("SO3 waypoint: [orientation=%s, angular velocity=%s]", getOrientation().toString(format), getAngularVelocity().toString(format));
   }
}
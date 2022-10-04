package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SO3WaypointReadOnly
{
   QuaternionReadOnly getOrientation();

   Vector3DReadOnly getAngularVelocity();

   default QuaternionBasics getOrientationCopy()
   {
      return new Quaternion(getOrientation());
   }

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

   default void getOrientation(QuaternionBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   default void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(getAngularVelocity());
   }

   default void get(SO3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   default boolean epsilonEquals(SO3WaypointReadOnly other, double epsilon)
   {
      boolean orientationMatches = getOrientation().epsilonEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default boolean geometricallyEquals(SO3WaypointReadOnly other, double epsilon)
   {
      boolean orientationMatches = getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getAngularVelocity().containsNaN();
   }

}
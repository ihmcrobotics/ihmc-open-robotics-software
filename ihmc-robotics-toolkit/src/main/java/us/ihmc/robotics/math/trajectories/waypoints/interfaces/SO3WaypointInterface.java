package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SO3WaypointInterface extends Transformable, Clearable
{
   abstract QuaternionReadOnly getOrientation();

   abstract void setOrientation(double x, double y, double z, double s);

   abstract Vector3DReadOnly getAngularVelocity();

   abstract void setAngularVelocity(double x, double y, double z);

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

   default void setOrientation(QuaternionReadOnly orientation)
   {
      setOrientation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
   }

   default void setOrientationToZero()
   {
      setOrientation(0.0, 0.0, 0.0, 1.0);
   }

   default void setOrientationToNaN()
   {
      setOrientation(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   default void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      setAngularVelocity(angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ());
   }

   default void setAngularVelocityToZero()
   {
      setAngularVelocity(0.0, 0.0, 0.0);
   }

   default void setAngularVelocityToNaN()
   {
      setAngularVelocity(Double.NaN, Double.NaN, Double.NaN);
   }

   default double orientationDistance(SO3WaypointInterface other)
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

   default void set(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setOrientation(orientation);
      setAngularVelocity(angularVelocity);
   }

   default void get(SO3WaypointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   default void get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   default boolean epsilonEquals(SO3WaypointInterface other, double epsilon)
   {
      boolean orientationMatches = getOrientation().epsilonEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default boolean geometricallyEquals(SO3WaypointInterface other, double epsilon)
   {
      boolean orientationMatches = getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default void set(SO3WaypointInterface other)
   {
      setOrientation(other.getOrientation());
      setAngularVelocity(other.getAngularVelocity());
   }

   @Override
   default void setToNaN()
   {
      setOrientationToNaN();
      setAngularVelocityToNaN();
   }

   @Override
   default void setToZero()
   {
      setOrientationToZero();
      setAngularVelocityToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getAngularVelocity().containsNaN();
   }

   default QuaternionBasics getOrientationCopy()
   {
      return new Quaternion(getOrientation());
   }

   default Vector3DBasics getAngularVelocityCopy()
   {
      return new Vector3D(getAngularVelocity());
   }
}

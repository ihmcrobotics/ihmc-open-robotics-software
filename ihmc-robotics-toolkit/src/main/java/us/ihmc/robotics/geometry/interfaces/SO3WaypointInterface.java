package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SO3WaypointInterface extends Transformable, Clearable
{
   public abstract QuaternionReadOnly getOrientation();

   public abstract void setOrientation(double x, double y, double z, double s);

   public abstract Vector3DReadOnly getAngularVelocity();

   public abstract void setAngularVelocity(double x, double y, double z);

   public default double getOrientationX()
   {
      return getOrientation().getX();
   }

   public default double getOrientationY()
   {
      return getOrientation().getY();
   }

   public default double getOrientationZ()
   {
      return getOrientation().getZ();
   }

   public default double getOrientationS()
   {
      return getOrientation().getS();
   }

   public default double getAngularVelocityX()
   {
      return getAngularVelocity().getX();
   }

   public default double getAngularVelocityY()
   {
      return getAngularVelocity().getY();
   }

   public default double getAngularVelocityZ()
   {
      return getAngularVelocity().getZ();
   }

   public default void setOrientation(QuaternionReadOnly orientation)
   {
      setOrientation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
   }

   public default void setOrientationToZero()
   {
      setOrientation(0.0, 0.0, 0.0, 1.0);
   }

   public default void setOrientationToNaN()
   {
      setOrientation(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   public default void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      setAngularVelocity(angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ());
   }

   public default void setAngularVelocityToZero()
   {
      setAngularVelocity(0.0, 0.0, 0.0);
   }

   public default void setAngularVelocityToNaN()
   {
      setAngularVelocity(Double.NaN, Double.NaN, Double.NaN);
   }

   public default double orientationDistance(SO3WaypointInterface other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   public default void getOrientation(QuaternionBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   public default void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(getAngularVelocity());
   }

   public default void set(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setOrientation(orientation);
      setAngularVelocity(angularVelocity);
   }

   public default void get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   public default boolean epsilonEquals(SO3WaypointInterface other, double epsilon)
   {
      boolean orientationMatches = getOrientation().epsilonEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   public default boolean geometricallyEquals(SO3WaypointInterface other, double epsilon)
   {
      boolean orientationMatches = getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   public default void set(SO3WaypointInterface other)
   {
      setOrientation(other.getOrientation());
      setAngularVelocity(other.getAngularVelocity());
   }

   @Override
   public default void setToNaN()
   {
      setOrientationToNaN();
      setAngularVelocityToNaN();
   }

   @Override
   public default void setToZero()
   {
      setOrientationToZero();
      setAngularVelocityToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getAngularVelocity().containsNaN();
   }
}

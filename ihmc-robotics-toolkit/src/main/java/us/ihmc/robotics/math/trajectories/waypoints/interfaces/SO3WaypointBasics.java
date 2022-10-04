package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SO3WaypointBasics extends Transformable, Clearable, SO3WaypointReadOnly
{
   void setOrientation(double x, double y, double z, double s);

   void setAngularVelocity(double x, double y, double z);

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

   default void set(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setOrientation(orientation);
      setAngularVelocity(angularVelocity);
   }

   default void set(SO3WaypointReadOnly other)
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
      return SO3WaypointReadOnly.super.containsNaN();
   }
}

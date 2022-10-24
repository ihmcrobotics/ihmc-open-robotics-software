package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;

public interface SO3WaypointBasics extends SO3WaypointReadOnly, Transformable, Clearable
{
   @Override
   QuaternionBasics getOrientation();

   @Override
   Vector3DBasics getAngularVelocity();

   @Override
   default void setToNaN()
   {
      getOrientation().setToNaN();
      getAngularVelocity().setToNaN();
   }

   @Override
   default void setToZero()
   {
      getOrientation().setToZero();
      getAngularVelocity().setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return SO3WaypointReadOnly.super.containsNaN();
   }

   default void set(Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      getOrientation().set(orientation);
      getAngularVelocity().set(angularVelocity);
   }

   default void set(SO3WaypointReadOnly other)
   {
      getOrientation().set(other.getOrientation());
      getAngularVelocity().set(other.getAngularVelocity());
   }

   @Override
   default void applyTransform(Transform transform)
   {
      getOrientation().applyTransform(transform);
      getAngularVelocity().applyTransform(transform);
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      getOrientation().applyInverseTransform(transform);
      getAngularVelocity().applyInverseTransform(transform);
   }
}

package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointBasics extends Transformable, Clearable, EuclideanWaypointReadOnly
{
   @Override
   Point3DBasics getPosition();

   @Override
   Vector3DBasics getLinearVelocity();

   @Override
   default void setToNaN()
   {
      getPosition().setToNaN();
      getLinearVelocity().setToNaN();
   }

   @Override
   default void setToZero()
   {
      getPosition().setToZero();
      getLinearVelocity().setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointReadOnly.super.containsNaN();
   }

   default void set(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      getPosition().set(position);
      getLinearVelocity().set(linearVelocity);
   }

   default void set(EuclideanWaypointReadOnly other)
   {
      getPosition().set(other.getPosition());
      getLinearVelocity().set(other.getLinearVelocity());
   }

   @Override
   default void applyTransform(Transform transform)
   {
      getPosition().applyTransform(transform);
      getLinearVelocity().applyTransform(transform);
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      getPosition().applyInverseTransform(transform);
      getLinearVelocity().applyInverseTransform(transform);
   }
}

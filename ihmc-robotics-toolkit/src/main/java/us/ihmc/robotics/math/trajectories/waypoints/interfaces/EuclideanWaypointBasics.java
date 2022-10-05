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

   @Deprecated
   default void setPosition(double x, double y, double z)
   {
      getPosition().set(x, y, z);
   }

   @Deprecated
   default void setLinearVelocity(double x, double y, double z)
   {
      getLinearVelocity().set(x, y, z);
   }

   @Deprecated
   default void setPosition(Point3DReadOnly position)
   {
      getPosition().set(position);
   }

   @Deprecated
   default void setPositionToZero()
   {
      getPosition().setToZero();
   }

   @Deprecated
   default void setPositionToNaN()
   {
      getPosition().setToNaN();
   }

   @Deprecated
   default void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      getLinearVelocity().set(linearVelocity);
   }

   @Deprecated
   default void setLinearVelocityToZero()
   {
      getLinearVelocity().setToZero();
   }

   @Deprecated
   default void setLinearVelocityToNaN()
   {
      getLinearVelocity().setToNaN();
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

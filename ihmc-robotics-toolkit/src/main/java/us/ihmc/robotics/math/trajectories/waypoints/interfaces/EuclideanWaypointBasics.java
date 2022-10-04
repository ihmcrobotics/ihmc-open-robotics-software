package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointBasics extends Transformable, Clearable, EuclideanWaypointReadOnly
{
   abstract void setPosition(double x, double y, double z);

   abstract void setLinearVelocity(double x, double y, double z);

   default void setPosition(Point3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   default void setPositionToZero()
   {
      setPosition(0.0, 0.0, 0.0);
   }

   default void setPositionToNaN()
   {
      setPosition(Double.NaN, Double.NaN, Double.NaN);
   }

   default void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   default void setLinearVelocityToZero()
   {
      setLinearVelocity(0.0, 0.0, 0.0);
   }

   default void setLinearVelocityToNaN()
   {
      setLinearVelocity(Double.NaN, Double.NaN, Double.NaN);
   }

   default void set(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
   }

   default void set(EuclideanWaypointReadOnly other)
   {
      setPosition(other.getPosition());
      setLinearVelocity(other.getLinearVelocity());
   }

   @Override
   default void setToNaN()
   {
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   @Override
   default void setToZero()
   {
      setPositionToZero();
      setLinearVelocityToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointReadOnly.super.containsNaN();
   }
}

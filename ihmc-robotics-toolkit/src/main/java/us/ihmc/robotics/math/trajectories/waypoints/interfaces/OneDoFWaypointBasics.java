package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;

public interface OneDoFWaypointBasics extends OneDoFWaypointReadOnly, Clearable
{
   void setPosition(double position);

   void setVelocity(double velocity);

   void setAcceleration(double acceleration);

   default void setPositionToZero()
   {
      setPosition(0.0);
   }

   default void setVelocityToZero()
   {
      setVelocity(0.0);
   }

   default void setAccelerationToZero()
   {
      setAcceleration(0.0);
   }

   default void setPositionToNaN()
   {
      setPosition(Double.NaN);
   }

   default void setVelocityToNaN()
   {
      setVelocity(Double.NaN);
   }

   default void setAccelerationToNaN()
   {
      setAcceleration(Double.NaN);
   }

   default void set(double position, double velocity, double acceleration)
   {
      setPosition(position);
      setVelocity(velocity);
      setAcceleration(acceleration);
   }

   default void set(OneDoFWaypointReadOnly other)
   {
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
      setAcceleration(other.getAcceleration());
   }

   @Override
   default void setToNaN()
   {
      setPositionToNaN();
      setVelocityToNaN();
      setAccelerationToNaN();
   }

   @Override
   default void setToZero()
   {
      setPositionToZero();
      setVelocityToZero();
      setAccelerationToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return OneDoFWaypointReadOnly.super.containsNaN();
   }
}

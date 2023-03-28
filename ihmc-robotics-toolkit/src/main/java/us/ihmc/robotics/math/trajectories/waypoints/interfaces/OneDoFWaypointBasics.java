package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;

public interface OneDoFWaypointBasics extends OneDoFWaypointReadOnly, Clearable
{
   void setPosition(double position);

   void setVelocity(double velocity);

   default void setPositionToZero()
   {
      setPosition(0.0);
   }

   default void setVelocityToZero()
   {
      setVelocity(0.0);
   }

   default void setPositionToNaN()
   {
      setPosition(Double.NaN);
   }

   default void setVelocityToNaN()
   {
      setVelocity(Double.NaN);
   }

   default void set(double position, double velocity)
   {
      setPosition(position);
      setVelocity(velocity);
   }

   default void set(OneDoFWaypointReadOnly other)
   {
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
   }

   @Override
   default void setToNaN()
   {
      setPositionToNaN();
      setVelocityToNaN();
   }

   @Override
   default void setToZero()
   {
      setPositionToZero();
      setVelocityToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return OneDoFWaypointReadOnly.super.containsNaN();
   }
}

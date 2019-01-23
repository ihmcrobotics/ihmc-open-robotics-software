package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

public interface OneDoFWaypointInterface
{
   abstract void setPosition(double position);

   abstract void setVelocity(double velocity);

   abstract double getPosition();

   abstract double getVelocity();

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

   default void get(OneDoFWaypointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(OneDoFWaypointInterface other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getPosition(), other.getPosition(), epsilon)
            && EuclidCoreTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon);
   }

   default void set(OneDoFWaypointInterface other)
   {
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
   }

   default void setToNaN()
   {
      setPositionToNaN();
      setVelocityToNaN();
   }

   default void setToZero()
   {
      setPositionToZero();
      setVelocityToZero();
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getPosition()) || Double.isNaN(getVelocity());
   }
}

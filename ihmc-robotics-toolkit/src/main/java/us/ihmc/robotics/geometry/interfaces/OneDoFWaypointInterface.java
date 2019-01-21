package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

public interface OneDoFWaypointInterface
{
   public abstract void setPosition(double position);

   public abstract void setVelocity(double velocity);

   public abstract double getPosition();

   public abstract double getVelocity();

   public default void setPositionToZero()
   {
      setPosition(0.0);
   }

   public default void setVelocityToZero()
   {
      setVelocity(0.0);
   }

   public default void setPositionToNaN()
   {
      setPosition(Double.NaN);
   }

   public default void setVelocityToNaN()
   {
      setVelocity(Double.NaN);
   }

   public default void set(double position, double velocity)
   {
      setPosition(position);
      setVelocity(velocity);
   }

   public default void get(OneDoFWaypointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   public default boolean epsilonEquals(OneDoFWaypointInterface other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getPosition(), other.getPosition(), epsilon)
            && EuclidCoreTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon);
   }

   public default void set(OneDoFWaypointInterface other)
   {
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
   }

   public default void setToNaN()
   {
      setPositionToNaN();
      setVelocityToNaN();
   }

   public default void setToZero()
   {
      setPositionToZero();
      setVelocityToZero();
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getPosition()) || Double.isNaN(getVelocity());
   }
}

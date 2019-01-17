package us.ihmc.robotics.geometry.interfaces;

public interface SE3WaypointInterface<T extends SE3WaypointInterface<T>> extends EuclideanWaypointInterface<T>, SO3WaypointInterface<T>
{
   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
      boolean so3Match = SO3WaypointInterface.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default boolean geometricallyEquals(T other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointInterface.super.geometricallyEquals(other, epsilon);
      boolean so3Match = SO3WaypointInterface.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default void set(T other)
   {
      EuclideanWaypointInterface.super.set(other);
      SO3WaypointInterface.super.set(other);
   }

   @Override
   default void setToNaN()
   {
      EuclideanWaypointInterface.super.setToNaN();
      SO3WaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      EuclideanWaypointInterface.super.setToZero();
      SO3WaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointInterface.super.containsNaN() || SO3WaypointInterface.super.containsNaN();
   }
}

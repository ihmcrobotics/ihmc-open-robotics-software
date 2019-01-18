package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;

public interface EuclideanTrajectoryPointInterface<T extends EuclideanTrajectoryPointInterface<T>> extends TrajectoryPointInterface, EuclideanWaypointInterface<T>
{
   public default void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   @Override
   default void set(T other)
   {
      setTime(other.getTime());
      EuclideanWaypointInterface.super.set(other);
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && EuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      EuclideanWaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      EuclideanWaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || EuclideanWaypointInterface.super.containsNaN();
   }
}
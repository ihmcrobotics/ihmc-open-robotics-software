package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;

public interface SO3TrajectoryPointInterface extends TrajectoryPointInterface, SO3WaypointInterface
{
   public default void set(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   default void set(SO3TrajectoryPointInterface other)
   {
      setTime(other.getTime());
      SO3WaypointInterface.super.set(other);
   }

   default void set(double time, SO3WaypointInterface waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default boolean epsilonEquals(SO3TrajectoryPointInterface other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && SO3WaypointInterface.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SO3WaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SO3WaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || SO3WaypointInterface.super.containsNaN();
   }
}

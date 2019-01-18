package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;

public interface SE3TrajectoryPointInterface<T extends SE3TrajectoryPointInterface<T>> extends TrajectoryPointInterface, SE3WaypointInterface<T>
{
   public default void set(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity,
                           Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   default void set(T other)
   {
      setTime(other.getTime());
      SE3WaypointInterface.super.set(other);
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && SE3WaypointInterface.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SE3WaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SE3WaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || SE3WaypointInterface.super.containsNaN();
   }
}
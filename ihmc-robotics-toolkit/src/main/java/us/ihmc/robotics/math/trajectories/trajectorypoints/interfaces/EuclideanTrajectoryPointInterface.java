package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;

public interface EuclideanTrajectoryPointInterface extends TrajectoryPointInterface, EuclideanWaypointInterface
{
   default void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(EuclideanTrajectoryPointInterface other)
   {
      setTime(other.getTime());
      EuclideanWaypointInterface.super.set(other);
   }

   default void set(double time, EuclideanWaypointInterface waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void get(EuclideanTrajectoryPointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(EuclideanTrajectoryPointInterface other, double epsilon)
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
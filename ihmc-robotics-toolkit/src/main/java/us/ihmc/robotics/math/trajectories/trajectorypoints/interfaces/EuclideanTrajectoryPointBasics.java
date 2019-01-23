package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;

public interface EuclideanTrajectoryPointBasics extends TrajectoryPointBasics, EuclideanWaypointBasics
{
   default void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(EuclideanTrajectoryPointBasics other)
   {
      setTime(other.getTime());
      EuclideanWaypointBasics.super.set(other);
   }

   default void set(double time, EuclideanWaypointBasics waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void get(EuclideanTrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(EuclideanTrajectoryPointBasics other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && EuclideanWaypointBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      EuclideanWaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      EuclideanWaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || EuclideanWaypointBasics.super.containsNaN();
   }
}
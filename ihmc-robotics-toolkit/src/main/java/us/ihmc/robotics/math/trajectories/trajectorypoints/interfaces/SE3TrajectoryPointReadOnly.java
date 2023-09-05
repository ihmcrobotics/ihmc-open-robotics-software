package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;

public interface SE3TrajectoryPointReadOnly extends TrajectoryPointReadOnly, SE3WaypointReadOnly, EuclideanTrajectoryPointReadOnly, SO3TrajectoryPointReadOnly
{
   @Override
   default boolean containsNaN()
   {
      return isTimeNaN() || SE3WaypointReadOnly.super.containsNaN();
   }

   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3TrajectoryPointReadOnly))
         return false;

      SE3TrajectoryPointReadOnly other = (SE3TrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.equals(getTime(), other.getTime()))
         return false;
      if (!getEuclideanWaypoint().equals(other.getEuclideanWaypoint()))
         return false;
      if (!getSO3Waypoint().equals(other.getSO3Waypoint()))
         return false;
      return true;
   }

   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3TrajectoryPointReadOnly))
         return false;

      SE3TrajectoryPointReadOnly other = (SE3TrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!getEuclideanWaypoint().epsilonEquals(other.getEuclideanWaypoint(), epsilon))
         return false;
      if (!getSO3Waypoint().epsilonEquals(other.getSO3Waypoint(), epsilon))
         return false;
      return true;
   }

   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SE3TrajectoryPointReadOnly))
         return false;

      SE3TrajectoryPointReadOnly other = (SE3TrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!getEuclideanWaypoint().geometricallyEquals(other.getEuclideanWaypoint(), epsilon))
         return false;
      if (!getSO3Waypoint().geometricallyEquals(other.getSO3Waypoint(), epsilon))
         return false;
      return true;
   }

   @Override
   default String toString(String format)
   {
      return String.format("SE3 trajectory point: [time=%s, position=%s, orientation=%s, linear velocity=%s, angular velocity=%s]",
                           EuclidCoreIOTools.getStringOf(null, format, getTime()),
                           EuclidCoreIOTools.getTuple3DString(format, getPosition()),
                           EuclidCoreIOTools.getTuple4DString(format, getOrientation()),
                           EuclidCoreIOTools.getTuple3DString(format, getLinearVelocity()),
                           EuclidCoreIOTools.getTuple3DString(format, getAngularVelocity()));
   }
}
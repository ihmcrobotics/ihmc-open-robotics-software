package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;

public interface SO3TrajectoryPointReadOnly extends TrajectoryPointReadOnly, SO3WaypointReadOnly
{
   @Override
   default boolean containsNaN()
   {
      return isTimeNaN() || SO3WaypointReadOnly.super.containsNaN();
   }

   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof SO3TrajectoryPointReadOnly))
         return false;

      SO3TrajectoryPointReadOnly other = (SO3TrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.equals(getTime(), other.getTime()))
         return false;
      if (!getOrientation().equals(other.getOrientation()))
         return false;
      if (!getAngularVelocity().equals(other.getAngularVelocity()))
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
      if (!(geometry instanceof SO3TrajectoryPointReadOnly))
         return false;

      SO3TrajectoryPointReadOnly other = (SO3TrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!getOrientation().epsilonEquals(other.getOrientation(), epsilon))
         return false;
      if (!getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon))
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
      if (!(geometry instanceof SO3TrajectoryPointReadOnly))
         return false;

      SO3TrajectoryPointReadOnly other = (SO3TrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!getOrientation().geometricallyEquals(other.getOrientation(), epsilon))
         return false;
      if (!getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   default String toString(String format)
   {
      return String.format("SO3 trajectory point: [time=%s, orientation=%s, angular velocity=%s]",
                           EuclidCoreIOTools.getStringOf(null, format, getTime()),
                           EuclidCoreIOTools.getTuple4DString(format, getOrientation()),
                           EuclidCoreIOTools.getTuple3DString(format, getAngularVelocity()));
   }
}
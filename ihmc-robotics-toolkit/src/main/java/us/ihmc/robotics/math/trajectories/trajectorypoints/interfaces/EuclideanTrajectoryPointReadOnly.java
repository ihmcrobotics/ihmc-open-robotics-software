package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;

public interface EuclideanTrajectoryPointReadOnly extends TrajectoryPointReadOnly, EuclideanWaypointReadOnly
{
   @Override
   default boolean containsNaN()
   {
      return isTimeNaN() || EuclideanWaypointReadOnly.super.containsNaN();
   }

   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof EuclideanTrajectoryPointReadOnly))
         return false;

      EuclideanTrajectoryPointReadOnly other = (EuclideanTrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.equals(getTime(), other.getTime()))
         return false;
      if (!getPosition().equals(other.getPosition()))
         return false;
      if (!getLinearVelocity().equals(other.getLinearVelocity()))
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
      if (!(geometry instanceof EuclideanTrajectoryPointReadOnly))
         return false;

      EuclideanTrajectoryPointReadOnly other = (EuclideanTrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!getPosition().epsilonEquals(other.getPosition(), epsilon))
         return false;
      if (!getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon))
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
      if (!(geometry instanceof EuclideanWaypointReadOnly))
         return false;

      EuclideanTrajectoryPointReadOnly other = (EuclideanTrajectoryPointReadOnly) geometry;

      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;
      if (!getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   default String toString(String format)
   {
      return String.format("Euclidean trajectory point: [time=%s, position=%s, linear velocity=%s]",
                           EuclidCoreIOTools.getStringOf(null, format, getTime()),
                           EuclidCoreIOTools.getTuple3DString(format, getPosition()),
                           EuclidCoreIOTools.getTuple3DString(format, getLinearVelocity()));
   }
}
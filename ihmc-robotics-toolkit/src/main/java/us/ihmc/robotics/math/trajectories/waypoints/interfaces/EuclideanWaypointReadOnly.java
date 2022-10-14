package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointReadOnly extends EuclidGeometry
{
   Point3DReadOnly getPosition();

   Vector3DReadOnly getLinearVelocity();

   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getLinearVelocity().containsNaN();
   }

   default double getPositionX()
   {
      return getPosition().getX();
   }

   default double getPositionY()
   {
      return getPosition().getY();
   }

   default double getPositionZ()
   {
      return getPosition().getZ();
   }

   default double getLinearVelocityX()
   {
      return getLinearVelocity().getX();
   }

   default double getLinearVelocityY()
   {
      return getLinearVelocity().getY();
   }

   default double getLinearVelocityZ()
   {
      return getLinearVelocity().getZ();
   }

   default double positionDistance(EuclideanWaypointReadOnly other)
   {
      return getPosition().distance(other.getPosition());
   }

   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof EuclideanWaypointReadOnly))
         return false;

      EuclideanWaypointReadOnly other = (EuclideanWaypointReadOnly) geometry;

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
      if (!(geometry instanceof EuclideanWaypointReadOnly))
         return false;

      EuclideanWaypointReadOnly other = (EuclideanWaypointReadOnly) geometry;

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

      EuclideanWaypointReadOnly other = (EuclideanWaypointReadOnly) geometry;

      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;
      if (!getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   default String toString(String format)
   {
      return String.format("Euclidean waypoint: [position=%s, linear velocity=%s]",
                           EuclidCoreIOTools.getTuple3DString(format, getPosition()),
                           EuclidCoreIOTools.getTuple3DString(format, getLinearVelocity()));
   }
}
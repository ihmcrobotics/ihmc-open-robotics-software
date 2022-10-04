package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointReadOnly
{
   Point3DReadOnly getPosition();

   Vector3DReadOnly getLinearVelocity();

   default Point3DBasics getPositionCopy()
   {
      return new Point3D(getPosition());
   }

   default Vector3DBasics getLinearVelocityCopy()
   {
      return new Vector3D(getLinearVelocity());
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

   default void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(getPosition());
   }

   default void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(getLinearVelocity());
   }

   default void get(Point3DBasics positionToPack, Vector3DBasics linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   default void get(EuclideanWaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(EuclideanWaypointReadOnly other, double epsilon)
   {
      boolean positionMatches = getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default boolean geometricallyEquals(EuclideanWaypointReadOnly other, double epsilon)
   {
      boolean positionMatches = getPosition().geometricallyEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getLinearVelocity().containsNaN();
   }
}
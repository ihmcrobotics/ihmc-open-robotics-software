package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointBasics extends Transformable, Clearable
{
   abstract Point3DReadOnly getPosition();

   abstract void setPosition(double x, double y, double z);

   abstract Vector3DReadOnly getLinearVelocity();

   abstract void setLinearVelocity(double x, double y, double z);

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

   default void setPosition(Point3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   default void setPositionToZero()
   {
      setPosition(0.0, 0.0, 0.0);
   }

   default void setPositionToNaN()
   {
      setPosition(Double.NaN, Double.NaN, Double.NaN);
   }

   default void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   default void setLinearVelocityToZero()
   {
      setLinearVelocity(0.0, 0.0, 0.0);
   }

   default void setLinearVelocityToNaN()
   {
      setLinearVelocity(Double.NaN, Double.NaN, Double.NaN);
   }

   default double positionDistance(EuclideanWaypointBasics other)
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

   default void set(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
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

   default boolean epsilonEquals(EuclideanWaypointBasics other, double epsilon)
   {
      boolean positionMatches = getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default boolean geometricallyEquals(EuclideanWaypointBasics other, double epsilon)
   {
      boolean positionMatches = getPosition().geometricallyEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default void set(EuclideanWaypointBasics other)
   {
      setPosition(other.getPosition());
      setLinearVelocity(other.getLinearVelocity());
   }

   @Override
   default void setToNaN()
   {
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   @Override
   default void setToZero()
   {
      setPositionToZero();
      setLinearVelocityToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getLinearVelocity().containsNaN();
   }

   default Point3DBasics getPositionCopy()
   {
      return new Point3D(getPosition());
   }

   default Vector3DBasics getLinearVelocityCopy()
   {
      return new Vector3D(getLinearVelocity());
   }
}

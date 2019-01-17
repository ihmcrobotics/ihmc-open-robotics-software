package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointInterface<T extends EuclideanWaypointInterface<T>> extends GeometryObject<T>
{
   public abstract Point3DReadOnly getPosition();

   public abstract void setPosition(double x, double y, double z);

   public abstract Vector3DReadOnly getLinearVelocity();

   public abstract void setLinearVelocity(double x, double y, double z);

   public default double getPositionX()
   {
      return getPosition().getX();
   }

   public default double getPositionY()
   {
      return getPosition().getY();
   }

   public default double getPositionZ()
   {
      return getPosition().getZ();
   }

   public default double getLinearVelocityX()
   {
      return getLinearVelocity().getX();
   }

   public default double getLinearVelocityY()
   {
      return getLinearVelocity().getY();
   }

   public default double getLinearVelocityZ()
   {
      return getLinearVelocity().getZ();
   }

   public default void setPosition(Point3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   public default void setPositionToZero()
   {
      setPosition(0.0, 0.0, 0.0);
   }

   public default void setPositionToNaN()
   {
      setPosition(Double.NaN, Double.NaN, Double.NaN);
   }

   public default void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   public default void setLinearVelocityToZero()
   {
      setLinearVelocity(0.0, 0.0, 0.0);
   }

   public default void setLinearVelocityToNaN()
   {
      setLinearVelocity(Double.NaN, Double.NaN, Double.NaN);
   }

   public default double positionDistance(T other)
   {
      return getPosition().distance(other.getPosition());
   }

   public default void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(getPosition());
   }

   public default void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(getLinearVelocity());
   }

   public default void set(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
   }

   public default void get(Point3DBasics positionToPack, Vector3DBasics linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public default boolean epsilonEquals(T other, double epsilon)
   {
      boolean positionMatches = getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   @Override
   public default boolean geometricallyEquals(T other, double epsilon)
   {
      boolean positionMatches = getPosition().geometricallyEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   @Override
   public default void set(T other)
   {
      setPosition(other.getPosition());
      setLinearVelocity(other.getLinearVelocity());
   }

   @Override
   public default void setToNaN()
   {
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   @Override
   public default void setToZero()
   {
      setPositionToZero();
      setLinearVelocityToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getLinearVelocity().containsNaN();
   }
}

package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameEuclideanWaypointInterface<T extends FrameEuclideanWaypointInterface<T>> extends EuclideanWaypointInterface<T>, FrameChangeable
{
   @Override
   public abstract FramePoint3DReadOnly getPosition();

   @Override
   public abstract FrameVector3DReadOnly getLinearVelocity();

   @Override
   public default void setPosition(Point3DReadOnly position)
   {
      if (position instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) position);
      }
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   @Override
   public default void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      if (linearVelocity instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) linearVelocity);
      }
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   @Override
   public default double positionDistance(T other)
   {
      return getPosition().distance(other.getPosition());
   }

   @Override
   public default void getPosition(Point3DBasics positionToPack)
   {
      if (positionToPack instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) positionToPack);
      }
      positionToPack.set(getPosition());
   }

   @Override
   public default void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      if (linearVelocityToPack instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) linearVelocityToPack);
      }
      linearVelocityToPack.set(getLinearVelocity());
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

   public default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   public default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }
}

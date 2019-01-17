package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameEuclideanWaypointInterface<T extends FrameEuclideanWaypointInterface<T>> extends EuclideanWaypointInterface<T>, FrameChangeable
{
   @Override
   public abstract FramePoint3DReadOnly getPosition();

   @Override
   public abstract FrameVector3DReadOnly getLinearVelocity();

   public default void setPosition(FramePoint3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   public default void setLinearVelocity(FrameVector3DReadOnly linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   @Override
   public default double positionDistance(T other)
   {
      return getPosition().distance(other.getPosition());
   }

   public default void getPosition(FramePoint3DBasics positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      positionToPack.set(getPosition());
   }

   public default void getLinearVelocity(FrameVector3DBasics linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      linearVelocityToPack.set(getLinearVelocity());
   }

   public default void getPositionIncludingFrame(FramePoint3DBasics positionToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
   }

   public default void getLinearVelocityIncludingFrame(FrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(getLinearVelocity());
   }

   public default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
   }

   public default void setIncludingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setReferenceFrame(position.getReferenceFrame());
      set(position, linearVelocity);
   }

   public default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(position, linearVelocity);
   }

   public default void get(FramePoint3DBasics positionToPack, FrameVector3DBasics linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   public default void getIncludingFrame(FramePoint3DBasics positionToPack, FrameVector3DBasics linearVelocityToPack)
   {
      getPositionIncludingFrame(positionToPack);
      getLinearVelocityIncludingFrame(linearVelocityToPack);
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

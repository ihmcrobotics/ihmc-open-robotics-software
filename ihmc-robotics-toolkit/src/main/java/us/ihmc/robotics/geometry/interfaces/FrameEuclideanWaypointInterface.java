package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameEuclideanWaypointInterface extends EuclideanWaypointInterface, FrameChangeable
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

   public default double positionDistance(FrameEuclideanWaypointInterface other)
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

   public default void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointInterface other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   public default void setIncludingFrame(FrameEuclideanWaypointInterface other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set(other);
   }

   public default void get(FrameEuclideanWaypointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   public default void getIncludingFrame(FrameEuclideanWaypointInterface otherToPack)
   {
      otherToPack.setIncludingFrame(this);
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

   public default boolean epsilonEquals(FrameEuclideanWaypointInterface other, double epsilon)
   {
      boolean positionMatches = getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   public default boolean geometricallyEquals(FrameEuclideanWaypointInterface other, double epsilon)
   {
      boolean positionMatches = getPosition().geometricallyEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   public default void set(FrameEuclideanWaypointInterface other)
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

   @Override
   public default FramePoint3DBasics getPositionCopy()
   {
      return new FramePoint3D(getPosition());
   }

   @Override
   public default FrameVector3DBasics getLinearVelocityCopy()
   {
      return new FrameVector3D(getLinearVelocity());
   }
}

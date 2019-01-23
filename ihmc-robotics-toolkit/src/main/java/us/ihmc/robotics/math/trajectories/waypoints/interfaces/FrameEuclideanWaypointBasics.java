package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameEuclideanWaypointBasics extends EuclideanWaypointBasics, FrameChangeable
{
   @Override
   abstract FramePoint3DReadOnly getPosition();

   @Override
   abstract FrameVector3DReadOnly getLinearVelocity();

   default void setPosition(FramePoint3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   default void setLinearVelocity(FrameVector3DReadOnly linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   default double positionDistance(FrameEuclideanWaypointBasics other)
   {
      return getPosition().distance(other.getPosition());
   }

   default void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      positionToPack.set(getPosition());
   }

   default void getLinearVelocity(FixedFrameVector3DBasics linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      linearVelocityToPack.set(getLinearVelocity());
   }

   default void getPositionIncludingFrame(FramePoint3DBasics positionToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
   }

   default void getLinearVelocityIncludingFrame(FrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(getLinearVelocity());
   }

   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
   }

   default void setIncludingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setReferenceFrame(position.getReferenceFrame());
      set(position, linearVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(position, linearVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointBasics other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameEuclideanWaypointBasics other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set(other);
   }

   default void get(FrameEuclideanWaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameEuclideanWaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   default void getIncludingFrame(FramePoint3DBasics positionToPack, FrameVector3DBasics linearVelocityToPack)
   {
      getPositionIncludingFrame(positionToPack);
      getLinearVelocityIncludingFrame(linearVelocityToPack);
   }

   default boolean epsilonEquals(FrameEuclideanWaypointBasics other, double epsilon)
   {
      boolean positionMatches = getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default boolean geometricallyEquals(FrameEuclideanWaypointBasics other, double epsilon)
   {
      boolean positionMatches = getPosition().geometricallyEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default void set(FrameEuclideanWaypointBasics other)
   {
      setPosition(other.getPosition());
      setLinearVelocity(other.getLinearVelocity());
   }

   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   @Override
   default FramePoint3DBasics getPositionCopy()
   {
      return new FramePoint3D(getPosition());
   }

   @Override
   default FrameVector3DBasics getLinearVelocityCopy()
   {
      return new FrameVector3D(getLinearVelocity());
   }
}

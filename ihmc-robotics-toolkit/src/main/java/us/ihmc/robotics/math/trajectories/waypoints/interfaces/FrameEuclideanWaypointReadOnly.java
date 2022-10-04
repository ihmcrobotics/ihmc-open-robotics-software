package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface FrameEuclideanWaypointReadOnly extends EuclideanWaypointReadOnly, ReferenceFrameHolder
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   FrameVector3DReadOnly getLinearVelocity();

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

   default void get(FixedFrameEuclideanWaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameEuclideanWaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default boolean epsilonEquals(FrameEuclideanWaypointReadOnly other, double epsilon)
   {
      boolean positionMatches = getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().epsilonEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }

   default boolean geometricallyEquals(FrameEuclideanWaypointReadOnly other, double epsilon)
   {
      boolean positionMatches = getPosition().geometricallyEquals(other.getPosition(), epsilon);
      boolean linearVelocityMatches = getLinearVelocity().geometricallyEquals(other.getLinearVelocity(), epsilon);
      return positionMatches && linearVelocityMatches;
   }
}
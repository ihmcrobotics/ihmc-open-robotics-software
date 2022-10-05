package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FrameEuclideanWaypointReadOnly extends EuclideanWaypointReadOnly, EuclidFrameGeometry
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   FrameVector3DReadOnly getLinearVelocity();

   @Deprecated
   @Override
   default FramePoint3DBasics getPositionCopy()
   {
      return new FramePoint3D(getPosition());
   }

   @Deprecated
   @Override
   default FrameVector3DBasics getLinearVelocityCopy()
   {
      return new FrameVector3D(getLinearVelocity());
   }

   @Deprecated
   default void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.set(getPosition());
   }

   @Deprecated
   default void getLinearVelocity(FixedFrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(getLinearVelocity());
   }

   @Deprecated
   default void getPositionIncludingFrame(FramePoint3DBasics positionToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
   }

   @Deprecated
   default void getLinearVelocityIncludingFrame(FrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(getLinearVelocity());
   }

   default void get(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics linearVelocityToPack)
   {
      positionToPack.set(getPosition());
      linearVelocityToPack.set(getLinearVelocity());
   }

   default void getIncludingFrame(FramePoint3DBasics positionToPack, FrameVector3DBasics linearVelocityToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
      linearVelocityToPack.setIncludingFrame(getLinearVelocity());
   }

   @Deprecated
   default void get(FixedFrameEuclideanWaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   @Deprecated
   default void getIncludingFrame(FrameEuclideanWaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   @Override
   default String toString(String format)
   {
      return EuclideanWaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
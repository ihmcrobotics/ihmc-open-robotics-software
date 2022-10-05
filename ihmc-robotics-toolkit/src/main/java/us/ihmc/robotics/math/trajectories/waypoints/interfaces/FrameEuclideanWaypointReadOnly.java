package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

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

   @Override
   default String toString(String format)
   {
      return EuclideanWaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameEuclideanWaypointBasics extends FixedFrameEuclideanWaypointBasics, FrameChangeable
{
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

   default void setIncludingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      position.checkReferenceFrameMatch(linearVelocity);
      setIncludingFrame(position.getReferenceFrame(), position, linearVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(position, linearVelocity);
   }

   default void setIncludingFrame(FrameEuclideanWaypointReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }
}

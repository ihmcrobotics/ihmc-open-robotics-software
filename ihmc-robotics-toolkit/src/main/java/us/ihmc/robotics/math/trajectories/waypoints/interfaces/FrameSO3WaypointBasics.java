package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameSO3WaypointBasics extends FixedFrameSO3WaypointBasics, FrameChangeable
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

   default void setIncludingFrame(FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setIncludingFrame(orientation.getReferenceFrame(), orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(orientation, angularVelocity);
   }

   default void setIncludingFrame(FrameSO3WaypointReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SO3WaypointReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }
}

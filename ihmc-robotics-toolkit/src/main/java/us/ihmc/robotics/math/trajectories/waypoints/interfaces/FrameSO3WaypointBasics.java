package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FrameSO3WaypointBasics extends FixedFrameSO3WaypointBasics, FrameChangeable
{
   default void setIncludingFrame(FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setReferenceFrame(orientation.getReferenceFrame());
      set(orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SO3WaypointReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSO3WaypointReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set(other);
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

}

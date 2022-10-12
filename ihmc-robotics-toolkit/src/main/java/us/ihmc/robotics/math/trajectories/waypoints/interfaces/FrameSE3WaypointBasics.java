package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameSE3WaypointBasics extends FixedFrameSE3WaypointBasics, FrameEuclideanWaypointBasics, FrameSO3WaypointBasics
{
   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointBasics.super.setToNaN(referenceFrame);
      FrameSO3WaypointBasics.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointBasics.super.setToZero(referenceFrame);
      FrameSO3WaypointBasics.super.setToZero(referenceFrame);
   }

   default void setIncludingFrame(FramePoint3DReadOnly position,
                                  FrameOrientation3DReadOnly orientation,
                                  FrameVector3DReadOnly linearVelocity,
                                  FrameVector3DReadOnly angularVelocity)
   {
      position.checkReferenceFrameMatch(orientation, linearVelocity, angularVelocity);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame,
                                  Point3DReadOnly position,
                                  Orientation3DReadOnly orientation,
                                  Vector3DReadOnly linearVelocity,
                                  Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      getEuclideanWaypoint().set(position, linearVelocity);
      getSO3Waypoint().set(orientation, angularVelocity);
   }

   default void setIncludingFrame(FrameSE3WaypointReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SE3WaypointReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }
}

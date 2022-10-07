package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FixedFrameSE3WaypointBasics extends FrameSE3WaypointReadOnly, FixedFrameEuclideanWaypointBasics, FixedFrameSO3WaypointBasics, SE3WaypointBasics
{
   @Override
   FixedFrameEuclideanWaypointBasics getEuclideanWaypoint();

   @Override
   FixedFrameSO3WaypointBasics getSO3Waypoint();

   @Override
   default FixedFramePoint3DBasics getPosition()
   {
      return getEuclideanWaypoint().getPosition();
   }

   @Override
   default FixedFrameQuaternionBasics getOrientation()
   {
      return getSO3Waypoint().getOrientation();
   }

   @Override
   default FixedFrameVector3DBasics getLinearVelocity()
   {
      return getEuclideanWaypoint().getLinearVelocity();
   }

   @Override
   default FixedFrameVector3DBasics getAngularVelocity()
   {
      return getSO3Waypoint().getAngularVelocity();
   }

   default void set(FramePoint3DReadOnly position,
                    FrameOrientation3DReadOnly orientation,
                    FrameVector3DReadOnly linearVelocity,
                    FrameVector3DReadOnly angularVelocity)
   {
      getEuclideanWaypoint().set(position, linearVelocity);
      getSO3Waypoint().set(orientation, angularVelocity);
   }

   default void set(FrameSE3WaypointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, SE3WaypointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }
}
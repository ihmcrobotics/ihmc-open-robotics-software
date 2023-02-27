package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FrameSE3WaypointReadOnly extends SE3WaypointReadOnly, FrameEuclideanWaypointReadOnly, FrameSO3WaypointReadOnly, EuclidFrameGeometry
{
   @Override
   FrameEuclideanWaypointReadOnly getEuclideanWaypoint();

   @Override
   FrameSO3WaypointReadOnly getSO3Waypoint();

   @Override
   default FramePoint3DReadOnly getPosition()
   {
      return getEuclideanWaypoint().getPosition();
   }

   @Override
   default FrameQuaternionReadOnly getOrientation()
   {
      return getSO3Waypoint().getOrientation();
   }

   @Override
   default FrameVector3DReadOnly getLinearVelocity()
   {
      return getEuclideanWaypoint().getLinearVelocity();
   }

   @Override
   default FrameVector3DReadOnly getAngularVelocity()
   {
      return getSO3Waypoint().getAngularVelocity();
   }

   default void getPose(FixedFramePose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   default void getPose(FramePose3DBasics poseToPack)
   {
      poseToPack.setIncludingFrame(getPosition(), getOrientation());
   }

   @Override
   default String toString(String format)
   {
      return SE3WaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
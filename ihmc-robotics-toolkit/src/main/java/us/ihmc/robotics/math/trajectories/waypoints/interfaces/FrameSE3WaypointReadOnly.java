package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
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

   @Deprecated
   default void get(FixedFrameSE3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   @Deprecated
   default void getIncludingFrame(FrameSE3WaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FixedFrameEuclideanWaypointBasics euclideanWaypointToPack, FixedFrameSO3WaypointBasics so3WaypointToPack)
   {
      euclideanWaypointToPack.set(getEuclideanWaypoint());
      so3WaypointToPack.set(getSO3Waypoint());
   }

   default void getIncludingFrame(FrameEuclideanWaypointBasics euclideanWaypointToPack, FrameSO3WaypointBasics so3WaypointToPack)
   {
      euclideanWaypointToPack.setIncludingFrame(getEuclideanWaypoint());
      so3WaypointToPack.setIncludingFrame(getSO3Waypoint());
   }

   default void get(FixedFramePoint3DBasics positionToPack,
                    FixedFrameOrientation3DBasics orientationToPack,
                    FixedFrameVector3DBasics linearVelocityToPack,
                    FixedFrameVector3DBasics angularVelocityToPack)
   {
      getEuclideanWaypoint().get(positionToPack, linearVelocityToPack);
      getSO3Waypoint().get(orientationToPack, angularVelocityToPack);
   }

   default void getIncludingFrame(FramePoint3DBasics positionToPack,
                                  FrameQuaternionBasics orientationToPack,
                                  FrameVector3DBasics linearVelocityToPack,
                                  FrameVector3DBasics angularVelocityToPack)
   {
      getEuclideanWaypoint().getIncludingFrame(positionToPack, linearVelocityToPack);
      getSO3Waypoint().getIncludingFrame(orientationToPack, angularVelocityToPack);
   }

   default void getPose(FixedFramePose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   default void getPoseIncludingFrame(FramePose3DBasics poseToPack)
   {
      poseToPack.setIncludingFrame(getPosition(), getOrientation());
   }

   @Override
   default String toString(String format)
   {
      return SE3WaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}
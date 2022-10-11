package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;

public interface FrameSE3TrajectoryPointBasics extends FixedFrameSE3TrajectoryPointBasics, SE3TrajectoryPointBasics, FrameSE3WaypointBasics,
      FrameEuclideanTrajectoryPointBasics, FrameSO3TrajectoryPointBasics
{
   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setTimeToNaN();
      FrameSE3WaypointBasics.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setTimeToZero();
      FrameSE3WaypointBasics.super.setToZero(referenceFrame);
   }

   default void setIncludingFrame(double time, FrameEuclideanWaypointReadOnly euclideanWaypoint, FrameSO3WaypointReadOnly so3Waypoint)
   {
      setTime(time);
      euclideanWaypoint.checkReferenceFrameMatch(so3Waypoint);
      setReferenceFrame(euclideanWaypoint.getReferenceFrame());
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void setIncludingFrame(double time, FrameSE3WaypointReadOnly waypoint)
   {
      setTime(time);
      setIncludingFrame(waypoint);
   }

   default void setIncludingFrame(double time,
                                  FramePoint3DReadOnly position,
                                  FrameOrientation3DReadOnly orientation,
                                  FrameVector3DReadOnly linearVelocity,
                                  FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame,
                                  double time,
                                  Point3DReadOnly position,
                                  Orientation3DReadOnly orientation,
                                  Vector3DReadOnly linearVelocity,
                                  Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SE3TrajectoryPointReadOnly trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameSE3WaypointBasics.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, SE3WaypointReadOnly waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameSE3TrajectoryPointReadOnly other)
   {
      setTime(other.getTime());
      FrameSE3WaypointBasics.super.setIncludingFrame(other);
   }
}

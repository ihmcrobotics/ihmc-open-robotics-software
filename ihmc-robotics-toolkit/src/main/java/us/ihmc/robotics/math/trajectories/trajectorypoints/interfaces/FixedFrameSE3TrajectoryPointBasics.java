package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public interface FixedFrameSE3TrajectoryPointBasics extends FrameSE3TrajectoryPointReadOnly, SE3TrajectoryPointBasics, FixedFrameSE3WaypointBasics,
      FixedFrameEuclideanTrajectoryPointBasics, FixedFrameSO3TrajectoryPointBasics
{
   default void set(double time,
                    FramePoint3DReadOnly position,
                    FrameOrientation3DReadOnly orientation,
                    FrameVector3DReadOnly linearVelocity,
                    FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   default void set(double time, FrameSE3WaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(double time, FrameEuclideanWaypointReadOnly euclideanWaypoint, FrameSO3WaypointReadOnly so3Waypoint)
   {
      setTime(time);
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void set(FrameSE3TrajectoryPointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, SE3TrajectoryPointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

}
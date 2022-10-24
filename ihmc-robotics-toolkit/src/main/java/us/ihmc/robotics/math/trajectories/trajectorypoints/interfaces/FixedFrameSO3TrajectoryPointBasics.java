package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public interface FixedFrameSO3TrajectoryPointBasics extends FrameSO3TrajectoryPointReadOnly, SO3TrajectoryPointBasics, FixedFrameSO3WaypointBasics
{
   default void set(double time, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   default void set(double time, FrameSO3WaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(FrameSO3TrajectoryPointReadOnly other)
   {
      set(other.getTime(), other);
   }

   default void set(ReferenceFrame referenceFrame, SO3TrajectoryPointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }
}
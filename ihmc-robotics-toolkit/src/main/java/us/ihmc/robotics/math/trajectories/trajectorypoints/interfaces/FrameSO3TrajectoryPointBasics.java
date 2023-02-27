package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;

public interface FrameSO3TrajectoryPointBasics extends FixedFrameSO3TrajectoryPointBasics, SO3TrajectoryPointBasics, FrameSO3WaypointBasics
{
   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setTimeToNaN();
      FrameSO3WaypointBasics.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setTimeToZero();
      FrameSO3WaypointBasics.super.setToZero(referenceFrame);
   }

   default void setIncludingFrame(double time, FrameSO3WaypointReadOnly waypoint)
   {
      setTime(time);
      setIncludingFrame(waypoint);
   }

   default void setIncludingFrame(double time, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SO3TrajectoryPointReadOnly trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameSO3WaypointBasics.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, SO3WaypointReadOnly waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameSO3TrajectoryPointReadOnly other)
   {
      setTime(other.getTime());
      FrameSO3WaypointBasics.super.setIncludingFrame(other);
   }
}

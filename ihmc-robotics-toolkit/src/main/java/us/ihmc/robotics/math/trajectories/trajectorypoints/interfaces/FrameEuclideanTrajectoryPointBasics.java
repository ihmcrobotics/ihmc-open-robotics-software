package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public interface FrameEuclideanTrajectoryPointBasics
      extends FixedFrameEuclideanTrajectoryPointBasics, EuclideanTrajectoryPointBasics, FrameEuclideanWaypointBasics
{
   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setTimeToNaN();
      FrameEuclideanWaypointBasics.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setTimeToZero();
      FrameEuclideanWaypointBasics.super.setToZero(referenceFrame);
   }

   default void setIncludingFrame(double time, FrameEuclideanWaypointReadOnly waypoint)
   {
      setTime(time);
      setIncludingFrame(waypoint);
   }

   default void setIncludingFrame(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      setIncludingFrame(position, linearVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, position, linearVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanTrajectoryPointReadOnly trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameEuclideanWaypointBasics.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, EuclideanWaypointReadOnly waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameEuclideanTrajectoryPointReadOnly other)
   {
      setTime(other.getTime());
      FrameEuclideanWaypointBasics.super.setIncludingFrame(other);
   }
}

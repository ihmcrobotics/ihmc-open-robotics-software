package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;

public interface FrameSE3TrajectoryPointReadOnly
      extends SE3TrajectoryPointReadOnly, FrameSE3WaypointReadOnly, FrameEuclideanTrajectoryPointReadOnly, FrameSO3TrajectoryPointReadOnly
{
   @Deprecated
   default void getIncludingFrame(FrameSE3TrajectoryPointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   @Deprecated
   default void get(FixedFrameSE3TrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameEuclideanTrajectoryPointBasics euclideanTrajectoryPointToPack, FrameSO3TrajectoryPointBasics so3TrajectoryPointToPack)
   {
      euclideanTrajectoryPointToPack.setIncludingFrame(this);
      so3TrajectoryPointToPack.setIncludingFrame(this);
   }

   default void get(FixedFrameEuclideanTrajectoryPointBasics euclideanTrajectoryPointToPack, FixedFrameSO3TrajectoryPointBasics so3TrajectoryPointToPack)
   {
      euclideanTrajectoryPointToPack.set(this);
      so3TrajectoryPointToPack.set(this);
   }

   @Override
   default String toString(String format)
   {
      return SE3TrajectoryPointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }

}
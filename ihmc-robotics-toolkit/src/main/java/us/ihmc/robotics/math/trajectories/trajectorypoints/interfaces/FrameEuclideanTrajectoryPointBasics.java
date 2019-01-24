package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;

public interface FrameEuclideanTrajectoryPointBasics extends EuclideanTrajectoryPointBasics, FrameEuclideanWaypointBasics
{
   default void set(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(double time, FrameEuclideanWaypointBasics waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void setIncludingFrame(double time, FrameEuclideanWaypointBasics waypoint)
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

   default void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanTrajectoryPointBasics trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameEuclideanWaypointBasics.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, EuclideanWaypointBasics waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameEuclideanTrajectoryPointBasics other)
   {
      setTime(other.getTime());
      FrameEuclideanWaypointBasics.super.setIncludingFrame(other);
   }

   default void set(FrameEuclideanTrajectoryPointBasics other)
   {
      setTime(other.getTime());
      FrameEuclideanWaypointBasics.super.set(other);
   }

   default void getIncludingFrame(FrameEuclideanTrajectoryPointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FrameEuclideanTrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(FrameEuclideanTrajectoryPointBasics other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameEuclideanWaypointBasics.super.epsilonEquals(other, epsilon);
   }
}

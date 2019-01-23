package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointInterface;

public interface FrameSE3TrajectoryPointInterface
      extends SE3TrajectoryPointInterface, FrameSE3WaypointInterface, FrameEuclideanTrajectoryPointInterface, FrameSO3TrajectoryPointInterface
{
   default void set(double time, FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                    FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   default void set(double time, FrameSE3WaypointInterface waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(double time, FrameEuclideanWaypointInterface euclideanWaypoint, FrameSO3WaypointInterface so3Waypoint)
   {
      setTime(time);
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void setIncludingFrame(double time, FrameEuclideanWaypointInterface euclideanWaypoint, FrameSO3WaypointInterface so3Waypoint)
   {
      setTime(time);
      euclideanWaypoint.checkReferenceFrameMatch(so3Waypoint);
      setReferenceFrame(euclideanWaypoint.getReferenceFrame());
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void setIncludingFrame(double time, FrameSE3WaypointInterface waypoint)
   {
      setTime(time);
      setIncludingFrame(waypoint);
   }

   default void setIncludingFrame(double time, FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                                  FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                  Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SE3TrajectoryPointInterface trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameSE3WaypointInterface.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, SE3WaypointInterface waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameSE3TrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameSE3WaypointInterface.super.setIncludingFrame(other);
   }

   default void set(FrameSE3TrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameSE3WaypointInterface.super.set(other);
   }

   default void getIncludingFrame(FrameSE3TrajectoryPointInterface otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FrameSE3TrajectoryPointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameEuclideanTrajectoryPointInterface euclideanTrajectoryPointToPack, FrameSO3TrajectoryPointInterface so3TrajectoryPointToPack)
   {
      getIncludingFrame(euclideanTrajectoryPointToPack);
      getIncludingFrame(so3TrajectoryPointToPack);
   }

   default void get(FrameEuclideanTrajectoryPointInterface euclideanTrajectoryPointToPack, FrameSO3TrajectoryPointInterface so3TrajectoryPointToPack)
   {
      get(euclideanTrajectoryPointToPack);
      get(so3TrajectoryPointToPack);
   }

   default boolean epsilonEquals(FrameSE3TrajectoryPointInterface other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameSE3WaypointInterface.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setTimeToNaN();
      FrameSE3WaypointInterface.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setTimeToZero();
      FrameSE3WaypointInterface.super.setToZero(referenceFrame);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SE3TrajectoryPointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SE3TrajectoryPointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || SE3TrajectoryPointInterface.super.containsNaN();
   }
}

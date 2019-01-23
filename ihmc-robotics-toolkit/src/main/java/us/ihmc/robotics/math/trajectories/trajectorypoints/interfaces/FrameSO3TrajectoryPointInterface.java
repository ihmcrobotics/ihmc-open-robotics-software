package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;

public interface FrameSO3TrajectoryPointInterface extends SO3TrajectoryPointInterface, FrameSO3WaypointInterface
{
   default void set(double time, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   default void set(double time, FrameSO3WaypointInterface waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void setIncludingFrame(double time, FrameSO3WaypointInterface waypoint)
   {
      setTime(time);
      setIncludingFrame(waypoint);
   }

   default void setIncludingFrame(double time, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SO3TrajectoryPointInterface trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameSO3WaypointInterface.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, SO3WaypointInterface waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameSO3TrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameSO3WaypointInterface.super.setIncludingFrame(other);
   }

   default void set(FrameSO3TrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameSO3WaypointInterface.super.set(other);
   }

   default void getIncludingFrame(FrameSO3TrajectoryPointInterface otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FrameSO3TrajectoryPointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean epsilonEquals(FrameSO3TrajectoryPointInterface other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameSO3WaypointInterface.super.epsilonEquals(other, epsilon);
   }
}

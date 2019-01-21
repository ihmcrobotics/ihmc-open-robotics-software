package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameEuclideanWaypointInterface;

public interface FrameEuclideanTrajectoryPointInterface extends EuclideanTrajectoryPointInterface, FrameEuclideanWaypointInterface
{
   default void set(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(double time, FrameEuclideanWaypointInterface waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void setIncludingFrame(double time, FrameEuclideanWaypointInterface waypoint)
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

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, EuclideanWaypointInterface waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void set(FrameEuclideanTrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameEuclideanWaypointInterface.super.set(other);
   }

   default boolean epsilonEquals(FrameEuclideanTrajectoryPointInterface other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameEuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
   }
}

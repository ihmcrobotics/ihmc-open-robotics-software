package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.interfaces.FrameSO3WaypointInterface;

public interface FrameSO3TrajectoryPointInterface extends SO3TrajectoryPointInterface, FrameSO3WaypointInterface
{
   public default void set(double time, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   public default void setIncludingFrame(double time, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(orientation, angularVelocity);
   }

   default void set(FrameSO3TrajectoryPointInterface other)
   {
      setTime(other.getTime());
      FrameSO3WaypointInterface.super.set(other);
   }

   default boolean epsilonEquals(FrameSO3TrajectoryPointInterface other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameSO3WaypointInterface.super.epsilonEquals(other, epsilon);
   }
}

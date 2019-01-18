package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.interfaces.FrameSO3WaypointInterface;

public interface FrameSO3TrajectoryPointInterface<T extends FrameSO3TrajectoryPointInterface<T>> extends SO3TrajectoryPointInterface<T>, FrameSO3WaypointInterface<T>
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

   @Override
   default void set(T other)
   {
      setTime(other.getTime());
      FrameSO3WaypointInterface.super.set(other);
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameSO3WaypointInterface.super.epsilonEquals(other, epsilon);
   }
}

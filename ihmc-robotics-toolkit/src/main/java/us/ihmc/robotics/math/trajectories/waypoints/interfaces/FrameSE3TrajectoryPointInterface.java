package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.interfaces.FrameSE3WaypointInterface;

public interface FrameSE3TrajectoryPointInterface<T extends FrameSE3TrajectoryPointInterface<T>>
      extends SE3TrajectoryPointInterface<T>, FrameSE3WaypointInterface<T>, FrameEuclideanTrajectoryPointInterface<T>, FrameSO3TrajectoryPointInterface<T>
{
   public default void set(double time, FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                           FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   public default void setIncludingFrame(double time, FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                                         FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   default void set(T other)
   {
      setTime(other.getTime());
      FrameSE3WaypointInterface.super.set(other);
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameSE3WaypointInterface.super.epsilonEquals(other, epsilon);
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

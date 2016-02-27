package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TransformableGeometryObjectInterface;

public abstract class FrameTrajectoryPoint<S extends TransformableGeometryObjectInterface<S> & TrajectoryPointInterface<S>, T extends FrameTrajectoryPoint<S, T>>
      extends FrameWaypoint<S, T> implements TrajectoryPointInterface<T>, TransformableGeometryObjectInterface<T>
{
   protected FrameTrajectoryPoint(S simpleTrajectoryPoint)
   {
      super(simpleTrajectoryPoint);
   }

   @Override
   public final void setTime(double time)
   {
      simpleWaypoint.setTime(time);
   }

   @Override
   public final void addTimeOffset(double timeOffsetToAdd)
   {
      simpleWaypoint.addTimeOffset(timeOffsetToAdd);
   }

   @Override
   public final void subtractTimeOffset(double timeOffsetToSubtract)
   {
      simpleWaypoint.subtractTimeOffset(timeOffsetToSubtract);
   }

   @Override
   public final void setTimeToZero()
   {
      simpleWaypoint.setTimeToZero();
   }

   @Override
   public final void setTimeToNaN()
   {
      simpleWaypoint.setTimeToNaN();
   }

   @Override
   public final double getTime()
   {
      return simpleWaypoint.getTime();
   }
}

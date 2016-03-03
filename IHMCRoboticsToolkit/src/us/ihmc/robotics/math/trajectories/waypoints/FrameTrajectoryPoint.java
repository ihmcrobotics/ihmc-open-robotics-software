package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;

public abstract class FrameTrajectoryPoint<T extends FrameTrajectoryPoint<T, S>, S extends TrajectoryPointInterface<S>>
      extends FrameWaypoint<T, S> implements TrajectoryPointInterface<T>
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

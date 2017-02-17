package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.GeometryObject;

public interface TrajectoryPointInterface<T extends TrajectoryPointInterface<T>> extends GeometryObject<T>
{
   public abstract void setTime(double time);

   public abstract void addTimeOffset(double timeOffsetToAdd);

   public abstract void subtractTimeOffset(double timeOffsetToSubtract);

   public abstract void setTimeToZero();

   public abstract void setTimeToNaN();

   public abstract double getTime();
}

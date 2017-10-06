package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.EpsilonComparable;

public interface TrajectoryPointListInterface<T extends TrajectoryPointListInterface<T, P>, P extends TrajectoryPointInterface<P>> extends EpsilonComparable<T>
{
   public abstract void clear();

   public abstract void addTrajectoryPoint(P trajectoryPoint);

   public abstract void set(T other);

   public abstract void addTimeOffset(double timeOffsetToAdd);

   public abstract void subtractTimeOffset(double timeOffsetToSubtract);

   public abstract int getNumberOfTrajectoryPoints();

   public abstract P getTrajectoryPoint(int trajectoryPointIndex);

   public abstract P getLastTrajectoryPoint();

   public abstract double getTrajectoryTime();
}

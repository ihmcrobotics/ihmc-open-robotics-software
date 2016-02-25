package us.ihmc.robotics.math.trajectories.waypoints;

public interface TrajectoryPointListInterface<T extends TrajectoryPointListInterface<T, P>, P extends TrajectoryPointInterface<P>>
{
   public abstract void clear();

   public abstract void addTrajectoryPoint(P trajectoryPoint);

   public abstract void set(T other);

   public abstract int getNumberOfTrajectoryPoints();

   public abstract P getTrajectoryPoint(int trajectoryPointIndex);

   public abstract P getLastTrajectoryPoint();

   public abstract double getTrajectoryTime();

   public abstract boolean epsilonEquals(T other, double epsilon);
}

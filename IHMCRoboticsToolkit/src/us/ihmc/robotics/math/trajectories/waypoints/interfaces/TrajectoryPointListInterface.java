package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface TrajectoryPointListInterface<P extends TrajectoryPointInterface<P>, T extends TrajectoryPointListInterface<P, T>>
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

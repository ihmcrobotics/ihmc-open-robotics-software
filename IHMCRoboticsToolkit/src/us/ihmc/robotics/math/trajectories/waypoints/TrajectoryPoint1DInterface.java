package us.ihmc.robotics.math.trajectories.waypoints;

public interface TrajectoryPoint1DInterface<T extends TrajectoryPoint1DInterface<T>> extends TrajectoryPointInterface<T>
{
   public abstract double getPosition();

   public abstract double getVelocity();
}

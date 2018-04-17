package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.providers.DoubleProvider;


public interface DoubleTrajectoryGenerator extends TrajectoryGenerator, DoubleProvider
{
   public abstract double getVelocity();
   public abstract double getAcceleration();
}

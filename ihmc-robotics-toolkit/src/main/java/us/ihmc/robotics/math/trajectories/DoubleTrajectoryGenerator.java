package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.providers.DoubleProvider;

public interface DoubleTrajectoryGenerator extends TrajectoryGenerator, DoubleProvider
{
   double getVelocity();
   double getAcceleration();
}

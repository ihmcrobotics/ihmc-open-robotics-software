package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public interface QuadrupedDataProvider
{
   public VectorProvider getDesiredVelocityProvider();
   public DoubleProvider getDesiredYawRateProvider();
}

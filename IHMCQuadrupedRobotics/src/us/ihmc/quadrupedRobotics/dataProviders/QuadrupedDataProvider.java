package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.quadrupedRobotics.trajectory.QuadrupedSwingTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public interface QuadrupedDataProvider
{
   public VectorProvider getDesiredVelocityProvider();
   public DoubleProvider getDesiredYawRateProvider();
   public FootSwitchProvider getFootSwitchProvider(RobotQuadrant robotQuadrant);
   public QuadrupedSwingTrajectoryGenerator getSwingTrajectoryGenerator(RobotQuadrant robotQuadrant);
}

package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;

public interface PoseTrajectoryGenerator extends PositionTrajectoryGenerator, OrientationTrajectoryGenerator, SE3ConfigurationProvider
{
   
}

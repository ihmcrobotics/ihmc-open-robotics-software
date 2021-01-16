package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;

public interface PoseTrajectoryGenerator extends FramePositionTrajectoryGenerator, FrameOrientationTrajectoryGenerator, SE3ConfigurationProvider
{
   
}

package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.robotics.trajectories.providers.FramePoseProvider;

public interface FixedFramePoseTrajectoryGenerator
      extends FixedFramePositionTrajectoryGenerator, FixedFrameOrientationTrajectoryGenerator, FramePoseProvider, PoseTrajectoryGenerator
{
   
}

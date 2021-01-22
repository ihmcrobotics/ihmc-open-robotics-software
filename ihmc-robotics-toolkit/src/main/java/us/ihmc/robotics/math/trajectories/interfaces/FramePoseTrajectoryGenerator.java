package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.robotics.trajectories.providers.FramePoseProvider;

public interface FramePoseTrajectoryGenerator
      extends FramePositionTrajectoryGenerator, FrameOrientationTrajectoryGenerator, FixedFramePoseTrajectoryGenerator, FrameChangeable
{

}

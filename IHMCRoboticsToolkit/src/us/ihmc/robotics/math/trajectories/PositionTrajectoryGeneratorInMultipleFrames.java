package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class PositionTrajectoryGeneratorInMultipleFrames extends TrajectoryGeneratorInMultipleFrames implements PositionTrajectoryGenerator
{
   public PositionTrajectoryGeneratorInMultipleFrames(boolean allowMultipleFrames, ReferenceFrame initialTrajectoryFrame)
   {
      super(allowMultipleFrames, initialTrajectoryFrame);
   }
}

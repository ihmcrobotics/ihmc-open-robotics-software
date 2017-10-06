package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public abstract class PositionTrajectoryGeneratorInMultipleFrames extends TrajectoryGeneratorInMultipleFrames implements PositionTrajectoryGenerator
{
   public PositionTrajectoryGeneratorInMultipleFrames(boolean allowMultipleFrames, ReferenceFrame initialTrajectoryFrame)
   {
      super(allowMultipleFrames, initialTrajectoryFrame);
   }
}

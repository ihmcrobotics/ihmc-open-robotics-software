package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class OrientationTrajectoryGeneratorInMultipleFrames extends TrajectoryGeneratorInMultipleFrames implements OrientationTrajectoryGenerator
{
   public OrientationTrajectoryGeneratorInMultipleFrames(boolean allowMultipleFrames, ReferenceFrame initialTrajectoryFrame)
   {
      super(allowMultipleFrames, initialTrajectoryFrame);
   }
}

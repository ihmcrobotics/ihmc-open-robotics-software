package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;

public interface SE3TrajectoryGenerator
{
   public abstract void initialize(FramePose initialPose, Twist initialTwist, FramePose finalDesiredPose);

   public abstract void updateFinalDesiredPose(FramePose finalDesiredPose);

   public abstract void compute(FramePose framePoseToPack, Twist twistToPack, SpatialAccelerationVector spatialAccelerationToPack, double deltaT);

   public abstract boolean isDone();
}

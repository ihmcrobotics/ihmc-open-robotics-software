package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;

public interface SE3TrajectoryGenerator
{
   public abstract void initialize(FramePose3D initialPose, Twist initialTwist, FramePose3D finalDesiredPose);

   public abstract void updateFinalDesiredPose(FramePose3D finalDesiredPose);

   public abstract void compute(FramePose3D framePoseToPack, Twist twistToPack, SpatialAccelerationVector spatialAccelerationToPack, double deltaT);

   public abstract boolean isDone();
}

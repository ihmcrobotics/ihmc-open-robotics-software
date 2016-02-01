package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public interface OrientationTrajectoryGenerator extends TrajectoryGenerator, OrientationProvider
{
   public abstract void packAngularVelocity(FrameVector angularVelocityToPack);

   public abstract void packAngularAcceleration(FrameVector angularAccelerationToPack);

   public abstract void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack);
}
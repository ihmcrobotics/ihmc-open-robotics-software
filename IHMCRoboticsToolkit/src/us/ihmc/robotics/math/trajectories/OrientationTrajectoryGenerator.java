package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public interface OrientationTrajectoryGenerator extends TrajectoryGenerator, OrientationProvider
{
   public abstract void getAngularVelocity(FrameVector3D angularVelocityToPack);

   public abstract void getAngularAcceleration(FrameVector3D angularAccelerationToPack);

   public default void getAngularData(FrameOrientation orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}
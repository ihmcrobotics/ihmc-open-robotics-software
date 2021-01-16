package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.FrameOrientationProvider;

public interface FrameOrientationTrajectoryGenerator extends TrajectoryGenerator, FrameOrientationProvider
{
   FrameVector3DReadOnly getAngularVelocity();

   FrameVector3DReadOnly getAngularAcceleration();

   default void getAngularData(FrameQuaternionBasics orientationToPack, FrameVector3DBasics angularVelocityToPack, FrameVector3DBasics angularAccelerationToPack)
   {
      orientationToPack.setIncludingFrame(getOrientation());
      angularVelocityToPack.setIncludingFrame(getAngularVelocity());
      angularAccelerationToPack.setIncludingFrame(getAngularAcceleration());
   }

   default void getAngularData(FixedFrameQuaternionBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack, FixedFrameVector3DBasics angularAccelerationToPack)
   {
      orientationToPack.setMatchingFrame(getOrientation());
      angularVelocityToPack.setMatchingFrame(getAngularVelocity());
      angularAccelerationToPack.setMatchingFrame(getAngularAcceleration());
   }
}
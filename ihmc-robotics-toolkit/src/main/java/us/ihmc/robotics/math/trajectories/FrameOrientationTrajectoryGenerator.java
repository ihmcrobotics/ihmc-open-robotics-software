package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.FrameOrientationProvider;

public interface FrameOrientationTrajectoryGenerator extends OrientationTrajectoryGenerator, FrameOrientationProvider
{
   @Override
   FrameVector3DReadOnly getAngularVelocity();

   @Override
   FrameVector3DReadOnly getAngularAcceleration();

   default void getAngularData(FrameOrientation3DBasics orientationToPack, FrameVector3DBasics angularVelocityToPack, FrameVector3DBasics angularAccelerationToPack)
   {
      orientationToPack.setReferenceFrame(getReferenceFrame());
      angularVelocityToPack.setReferenceFrame(getReferenceFrame());
      angularAccelerationToPack.setReferenceFrame(getReferenceFrame());
      OrientationTrajectoryGenerator.super.getAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);
   }

   default void getAngularData(FixedFrameOrientation3DBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack, FixedFrameVector3DBasics angularAccelerationToPack)
   {
      orientationToPack.setMatchingFrame(getOrientation());
      angularVelocityToPack.setMatchingFrame(getAngularVelocity());
      angularAccelerationToPack.setMatchingFrame(getAngularAcceleration());
   }
}
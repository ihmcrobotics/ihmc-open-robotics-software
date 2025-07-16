package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface FixedFrameOrientationTrajectoryGenerator extends OrientationTrajectoryGenerator, ReferenceFrameHolder
{
   default ReferenceFrame getReferenceFrame()
   {
      return getOrientation().getReferenceFrame();
   }

   @Override
   FrameOrientation3DReadOnly getOrientation();

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
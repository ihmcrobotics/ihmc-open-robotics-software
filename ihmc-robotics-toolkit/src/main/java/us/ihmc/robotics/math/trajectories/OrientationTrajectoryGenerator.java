package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public interface OrientationTrajectoryGenerator extends TrajectoryGenerator, OrientationProvider
{
   default void getAngularVelocity(FrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setReferenceFrame(this.getReferenceFrame());
      getAngularVelocity((FixedFrameVector3DBasics) angularVelocityToPack);
   }

   default void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setMatchingFrame(getAngularVelocity());
   }

   FrameVector3DReadOnly getAngularVelocity();

   default void getAngularAcceleration(FrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.setReferenceFrame(this.getReferenceFrame());
      getAngularAcceleration((FixedFrameVector3DBasics) angularAccelerationToPack);
   }

   default void getAngularAcceleration(FixedFrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.setMatchingFrame(getAngularAcceleration());
   }

   FrameVector3DReadOnly getAngularAcceleration();

   default void getAngularData(FrameQuaternionBasics orientationToPack, FrameVector3DBasics angularVelocityToPack, FrameVector3DBasics angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   default void getAngularData(FixedFrameQuaternionBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack, FixedFrameVector3DBasics angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}
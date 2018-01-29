package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface CenterOfMassDataHolderReadOnly
{
   void getCenterOfMassVelocity(FrameVector3D centerOfMassVelocityToPack);
   void getCenterOfMassAcceleration(FrameVector3D centerOfMassAccelerationToPack);
}

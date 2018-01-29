package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface CenterOfMassDataHolderReadOnly
{
   public abstract void getCenterOfMassVelocity(FrameVector3D centerOfMassVelocityToPack);

}

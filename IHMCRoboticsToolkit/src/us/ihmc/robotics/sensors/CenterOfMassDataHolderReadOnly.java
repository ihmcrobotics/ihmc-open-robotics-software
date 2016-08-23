package us.ihmc.robotics.sensors;

import us.ihmc.robotics.geometry.FrameVector;

public interface CenterOfMassDataHolderReadOnly
{
   public abstract void getCenterOfMassVelocity(FrameVector centerOfMassVelocityToPack);

}

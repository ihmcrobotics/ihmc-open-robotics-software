package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;

public interface OrientationProvider
{
   public abstract void getOrientation(FrameQuaternion orientationToPack);
}
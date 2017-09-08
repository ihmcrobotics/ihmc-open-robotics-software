package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface VectorProvider
{
   public abstract void get(FrameVector3D frameVectorToPack);
}

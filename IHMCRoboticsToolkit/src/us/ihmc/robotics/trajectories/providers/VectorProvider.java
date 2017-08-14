package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameVector3D;

public interface VectorProvider
{
   public abstract void get(FrameVector3D frameVectorToPack);
}

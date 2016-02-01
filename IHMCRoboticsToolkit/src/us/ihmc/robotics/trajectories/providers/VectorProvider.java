package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameVector;

public interface VectorProvider
{
   public abstract void get(FrameVector frameVectorToPack);
}

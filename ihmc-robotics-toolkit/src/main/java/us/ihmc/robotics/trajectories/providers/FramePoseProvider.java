package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePose3D;

public interface FramePoseProvider
{
   public abstract void getPose(FramePose3D framePoseToPack);
}
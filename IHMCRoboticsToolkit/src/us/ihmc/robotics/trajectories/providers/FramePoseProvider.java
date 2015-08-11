package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePose;

public interface FramePoseProvider
{
   public abstract void get(FramePose framePoseToPack);
}
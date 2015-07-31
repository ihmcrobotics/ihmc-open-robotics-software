package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.geometry.FramePoint2d;

public interface ICPProvider
{
   public abstract void getICP(FramePoint2d pointToPack);
}
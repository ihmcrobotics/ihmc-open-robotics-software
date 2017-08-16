package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;

public interface PositionProvider
{
   public abstract void getPosition(FramePoint3D positionToPack);
}

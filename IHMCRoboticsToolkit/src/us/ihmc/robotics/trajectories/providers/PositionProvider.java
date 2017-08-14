package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint3D;

public interface PositionProvider
{
   public abstract void getPosition(FramePoint3D positionToPack);
}

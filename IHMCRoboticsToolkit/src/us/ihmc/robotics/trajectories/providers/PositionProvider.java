package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint;

public interface PositionProvider
{
   public abstract void getPosition(FramePoint positionToPack);
}

package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint;

public class SettablePositionProvider implements PositionProvider
{
   private final FramePoint framePoint;

   public SettablePositionProvider()
   {
      framePoint = new FramePoint();
   }

   public void set(FramePoint framePoint)
   {
      this.framePoint.setIncludingFrame(framePoint);
   }

   @Override
   public void get(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(framePoint);
   }
}

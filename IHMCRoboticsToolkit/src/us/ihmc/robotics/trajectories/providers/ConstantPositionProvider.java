package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint;

public class ConstantPositionProvider implements PositionProvider
{
   private final FramePoint framePoint;

   public ConstantPositionProvider(FramePoint position)
   {
      this.framePoint = new FramePoint(position);
   }

   public void get(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(framePoint);
   }

}

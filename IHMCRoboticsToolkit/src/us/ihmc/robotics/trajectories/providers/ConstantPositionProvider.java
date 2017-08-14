package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint3D;

public class ConstantPositionProvider implements PositionProvider
{
   private final FramePoint3D framePoint;

   public ConstantPositionProvider(FramePoint3D position)
   {
      this.framePoint = new FramePoint3D(position);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(framePoint);
   }

}

package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;

public class SettablePositionProvider implements PositionProvider
{
   private final FramePoint3D framePoint;

   public SettablePositionProvider()
   {
      framePoint = new FramePoint3D();
   }

   public void set(FramePoint3D framePoint)
   {
      this.framePoint.setIncludingFrame(framePoint);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(framePoint);
   }
}

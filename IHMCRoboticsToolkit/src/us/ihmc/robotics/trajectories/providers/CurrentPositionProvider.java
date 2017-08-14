package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CurrentPositionProvider implements PositionProvider
{
   private final ReferenceFrame referenceFrame;

   public CurrentPositionProvider(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void getPosition(FramePoint3D framePointToPack)
   {
      framePointToPack.setToZero(referenceFrame);
   }
}

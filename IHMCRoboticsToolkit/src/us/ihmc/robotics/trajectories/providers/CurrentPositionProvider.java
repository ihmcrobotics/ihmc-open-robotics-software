package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FramePoint;

public class CurrentPositionProvider implements PositionProvider
{
   private final ReferenceFrame referenceFrame;

   public CurrentPositionProvider(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void getPosition(FramePoint framePointToPack)
   {
      framePointToPack.setToZero(referenceFrame);
   }
}

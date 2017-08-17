package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

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

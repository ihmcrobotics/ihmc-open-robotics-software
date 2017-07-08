package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FramePoint3D;

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

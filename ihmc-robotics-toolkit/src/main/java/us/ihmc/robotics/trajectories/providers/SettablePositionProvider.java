package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class SettablePositionProvider implements PositionProvider
{
   private final FramePoint3D framePoint;

   public SettablePositionProvider()
   {
      framePoint = new FramePoint3D();
   }

   public void set(FramePoint3DReadOnly framePoint)
   {
      this.framePoint.setIncludingFrame(framePoint);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return framePoint.getReferenceFrame();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return framePoint;
   }
}

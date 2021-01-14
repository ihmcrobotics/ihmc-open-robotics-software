package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

/**
 * This class is only used in tests.
 */
@Deprecated
public class ConstantPositionProvider implements PositionProvider
{
   private final FramePoint3D framePoint;

   public ConstantPositionProvider(FramePoint3D position)
   {
      this.framePoint = new FramePoint3D(position);
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

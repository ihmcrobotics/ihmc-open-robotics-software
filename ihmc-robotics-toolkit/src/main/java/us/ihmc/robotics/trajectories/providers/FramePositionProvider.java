package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface FramePositionProvider extends ReferenceFrameHolder
{
   default ReferenceFrame getReferenceFrame()
   {
      return getPosition().getReferenceFrame();
   }

   FramePoint3DReadOnly getPosition();
}

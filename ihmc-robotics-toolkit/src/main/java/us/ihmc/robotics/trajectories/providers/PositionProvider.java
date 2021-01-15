package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface PositionProvider extends ReferenceFrameHolder
{
   default void getPosition(FramePoint3DBasics positionToPack)
   {
      positionToPack.setReferenceFrame(this.getReferenceFrame());
      getPosition((FixedFramePoint3DBasics) positionToPack);
   }

   default void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.setMatchingFrame(getPosition());
   }

   default ReferenceFrame getReferenceFrame()
   {
      return getPosition().getReferenceFrame();
   }

   FramePoint3DReadOnly getPosition();
}

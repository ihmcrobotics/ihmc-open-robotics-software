package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;

public interface FrameSTPRamp3DBasics extends FixedFrameSTPRamp3DBasics, FrameRamp3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, STPRamp3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSTPRamp3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}

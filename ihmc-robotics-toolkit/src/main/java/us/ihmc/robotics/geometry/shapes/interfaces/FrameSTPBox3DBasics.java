package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DBasics;

public interface FrameSTPBox3DBasics extends FixedFrameSTPBox3DBasics, FrameBox3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, STPBox3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSTPBox3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}

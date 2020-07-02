package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRamp3DBasics;

public interface FixedFrameSTPRamp3DBasics extends STPRamp3DBasics, FixedFrameRamp3DBasics, FrameSTPRamp3DReadOnly
{
   default void set(ReferenceFrame referenceFrame, STPRamp3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameSTPRamp3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }
}

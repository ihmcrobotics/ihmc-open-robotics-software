package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBox3DBasics;

public interface FixedFrameSTPBox3DBasics extends STPBox3DBasics, FixedFrameBox3DBasics, FrameSTPBox3DReadOnly
{
   default void set(ReferenceFrame referenceFrame, STPBox3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameSTPBox3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }
}

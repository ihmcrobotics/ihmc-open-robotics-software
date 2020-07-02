package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DBasics;

public interface FrameSTPCapsule3DBasics extends FixedFrameSTPCapsule3DBasics, FrameCapsule3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, STPCapsule3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSTPCapsule3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}

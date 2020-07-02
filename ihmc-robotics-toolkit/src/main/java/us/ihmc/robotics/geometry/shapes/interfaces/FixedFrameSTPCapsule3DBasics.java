package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameCapsule3DBasics;

public interface FixedFrameSTPCapsule3DBasics extends STPCapsule3DBasics, FixedFrameCapsule3DBasics, FrameSTPCapsule3DReadOnly
{
   default void set(ReferenceFrame referenceFrame, STPCapsule3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameSTPCapsule3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }
}

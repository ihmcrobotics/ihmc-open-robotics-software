package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameCylinder3DBasics;

public interface FixedFrameSTPCylinder3DBasics extends STPCylinder3DBasics, FixedFrameCylinder3DBasics, FrameSTPCylinder3DReadOnly
{
   default void set(ReferenceFrame referenceFrame, STPCylinder3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameSTPCylinder3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }
}

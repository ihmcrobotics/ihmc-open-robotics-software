package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DBasics;

public interface FrameSTPCylinder3DBasics extends FixedFrameSTPCylinder3DBasics, FrameCylinder3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, STPCylinder3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSTPCylinder3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}

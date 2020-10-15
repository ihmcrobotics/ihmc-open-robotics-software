package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DBasics;

/**
 * Write and read interface for a capsule that implements the sphere-torus-patches (STP) method to
 * make shapes strictly convex and that is expressed in a modifiable reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
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

package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameCapsule3DBasics;

/**
 * Write and read interface for a capsule that implements the sphere-torus-patches (STP) method to
 * make shapes strictly convex and that is expressed in an unmodifiable reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
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

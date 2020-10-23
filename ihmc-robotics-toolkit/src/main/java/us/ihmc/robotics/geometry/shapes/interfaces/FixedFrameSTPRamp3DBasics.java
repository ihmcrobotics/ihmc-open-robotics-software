package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRamp3DBasics;

/**
 * Write and read interface for a ramp that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex and that is expressed in an unmodifiable reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
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

package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;

/**
 * Write and read interface for a ramp that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex and that is expressed in a modifiable reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface FrameSTPRamp3DBasics extends FixedFrameSTPRamp3DBasics, FrameRamp3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, STPRamp3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSTPRamp3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}

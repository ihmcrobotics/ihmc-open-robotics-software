package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DBasics;

/**
 * Write and read interface for a cylinder that implements the sphere-torus-patches (STP) method to
 * make shapes strictly convex and that is expressed in a modifiable reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
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

package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DBasics;

public interface STPRamp3DBasics extends STPShape3DBasics, STPRamp3DReadOnly, Ramp3DBasics
{
   default void set(STPRamp3DReadOnly other)
   {
      Ramp3DBasics.super.set(other);
      setMargins(other.getMinimumMargin(), other.getMaximumMargin());
   }
}

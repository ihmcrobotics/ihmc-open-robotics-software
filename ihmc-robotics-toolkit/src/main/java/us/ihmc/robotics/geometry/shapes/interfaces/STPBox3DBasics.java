package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;

public interface STPBox3DBasics extends STPShape3DBasics, STPBox3DReadOnly, Box3DBasics
{
   default void set(STPBox3DReadOnly other)
   {
      Box3DBasics.super.set(other);
      setMargins(other.getMinimumMargin(), other.getMaximumMargin());
   }
}

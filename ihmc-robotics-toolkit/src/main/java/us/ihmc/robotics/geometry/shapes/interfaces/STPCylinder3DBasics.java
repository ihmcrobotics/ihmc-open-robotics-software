package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DBasics;

public interface STPCylinder3DBasics extends STPShape3DBasics, STPCylinder3DReadOnly, Cylinder3DBasics
{
   default void set(STPCylinder3DReadOnly other)
   {
      Cylinder3DBasics.super.set(other);
      setMargins(other.getMinimumMargin(), other.getMaximumMargin());
   }
}

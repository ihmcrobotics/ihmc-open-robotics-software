package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DBasics;

/**
 * Write and read interface for a cylinder that implements the sphere-torus-patches (STP) method to
 * make shapes strictly convex.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface STPCylinder3DBasics extends STPShape3DBasics, STPCylinder3DReadOnly, Cylinder3DBasics
{
   default void set(STPCylinder3DReadOnly other)
   {
      Cylinder3DBasics.super.set(other);
      setMargins(other.getMinimumMargin(), other.getMaximumMargin());
   }
}

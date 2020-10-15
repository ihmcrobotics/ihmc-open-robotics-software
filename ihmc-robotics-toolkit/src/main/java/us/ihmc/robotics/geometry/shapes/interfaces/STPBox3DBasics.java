package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;

/**
 * Write and read interface for a box that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface STPBox3DBasics extends STPShape3DBasics, STPBox3DReadOnly, Box3DBasics
{
   default void set(STPBox3DReadOnly other)
   {
      Box3DBasics.super.set(other);
      setMargins(other.getMinimumMargin(), other.getMaximumMargin());
   }
}

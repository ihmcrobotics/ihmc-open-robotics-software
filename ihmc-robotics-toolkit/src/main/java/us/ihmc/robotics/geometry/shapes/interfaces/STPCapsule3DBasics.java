package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DBasics;

/**
 * Write and read interface for a capsule that implements the sphere-torus-patches (STP) method to
 * make shapes strictly convex.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface STPCapsule3DBasics extends STPShape3DBasics, STPCapsule3DReadOnly, Capsule3DBasics
{
   default void set(STPCapsule3DReadOnly other)
   {
      Capsule3DBasics.super.set(other);
      setMargins(other.getMinimumMargin(), other.getMaximumMargin());
   }
}

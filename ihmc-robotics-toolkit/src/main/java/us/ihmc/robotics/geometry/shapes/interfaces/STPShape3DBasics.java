package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;

/**
 * Write and read interface for shapes that implement the sphere-torus-patches (STP) method to make
 * shapes strictly convex.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface STPShape3DBasics extends STPShape3DReadOnly, Shape3DBasics
{
   /**
    * Sets the parameters for the "smoothness" of this shape.
    * 
    * @param minimumMargin the minimum distance between the original shape and the STP shape.
    * @param maximumMargin the maximum distance between the original shape and the STP shape. A greater
    *                      value translates to a smoother STP shape but that is less accurate
    *                      representation of the original shape.
    * @see STPShape3DReadOnly
    * @throws IllegalArgumentException if {@code minimumMargin > maximumMargin}.
    */
   void setMargins(double minimumMargin, double maximumMargin);
}

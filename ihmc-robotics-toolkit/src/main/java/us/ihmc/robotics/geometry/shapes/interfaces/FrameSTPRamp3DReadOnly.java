package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;

/**
 * Read-only interface for a ramp that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex and that is expressed in a reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface FrameSTPRamp3DReadOnly extends STPRamp3DReadOnly, FrameRamp3DReadOnly
{
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameRamp3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      // TODO Naive implementation of the bounding box. It is guaranteed to contain the shape but it is not the tightest bounding box.
      FrameRamp3DReadOnly.super.getBoundingBox(destinationFrame, boundingBoxToPack);
      boundingBoxToPack.getMinPoint().sub(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
      boundingBoxToPack.getMaxPoint().add(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
   }

   default boolean equals(FrameSTPRamp3DReadOnly other)
   {
      return FrameRamp3DReadOnly.super.equals(other) && STPRamp3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(FrameSTPRamp3DReadOnly other, double epsilon)
   {
      return FrameRamp3DReadOnly.super.epsilonEquals(other, epsilon) && STPRamp3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSTPRamp3DReadOnly other, double epsilon)
   {
      return FrameRamp3DReadOnly.super.geometricallyEquals(other, epsilon) && STPRamp3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;

/**
 * Read-only interface for a box that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex and that is expressed in a reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface FrameSTPBox3DReadOnly extends STPBox3DReadOnly, FrameBox3DReadOnly
{
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameBox3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      // TODO Naive implementation of the bounding box. It is guaranteed to contain the shape but it is not the tightest bounding box.
      FrameBox3DReadOnly.super.getBoundingBox(destinationFrame, boundingBoxToPack);
      boundingBoxToPack.getMinPoint().sub(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
      boundingBoxToPack.getMaxPoint().add(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
   }

   default boolean equals(FrameSTPBox3DReadOnly other)
   {
      return FrameBox3DReadOnly.super.equals(other) && STPBox3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(FrameSTPBox3DReadOnly other, double epsilon)
   {
      return FrameBox3DReadOnly.super.epsilonEquals(other, epsilon) && STPBox3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSTPBox3DReadOnly other, double epsilon)
   {
      return FrameBox3DReadOnly.super.geometricallyEquals(other, epsilon) && STPBox3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

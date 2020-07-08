package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;

/**
 * Read-only interface for a capsule that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex and that is expressed in a reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface FrameSTPCapsule3DReadOnly extends STPCapsule3DReadOnly, FrameCapsule3DReadOnly
{
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameCapsule3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      // TODO Naive implementation of the bounding box. It is guaranteed to contain the shape but it is not the tightest bounding box.
      FrameCapsule3DReadOnly.super.getBoundingBox(destinationFrame, boundingBoxToPack);
      boundingBoxToPack.getMinPoint().sub(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
      boundingBoxToPack.getMaxPoint().add(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
   }

   default boolean equals(FrameSTPCapsule3DReadOnly other)
   {
      return FrameCapsule3DReadOnly.super.equals(other) && STPCapsule3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(FrameSTPCapsule3DReadOnly other, double epsilon)
   {
      return FrameCapsule3DReadOnly.super.epsilonEquals(other, epsilon) && STPCapsule3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSTPCapsule3DReadOnly other, double epsilon)
   {
      return FrameCapsule3DReadOnly.super.geometricallyEquals(other, epsilon) && STPCapsule3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

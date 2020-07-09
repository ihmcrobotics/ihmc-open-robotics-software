package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;

/**
 * Read-only interface for a cylinder that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex and that is expressed in a reference frame.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface FrameSTPCylinder3DReadOnly extends STPCylinder3DReadOnly, FrameCylinder3DReadOnly
{
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameCylinder3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      // TODO Naive implementation of the bounding box. It is guaranteed to contain the shape but it is not the tightest bounding box.
      FrameCylinder3DReadOnly.super.getBoundingBox(destinationFrame, boundingBoxToPack);
      boundingBoxToPack.getMinPoint().sub(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
      boundingBoxToPack.getMaxPoint().add(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
   }

   default boolean equals(FrameSTPCylinder3DReadOnly other)
   {
      return FrameCylinder3DReadOnly.super.equals(other) && STPCylinder3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(FrameSTPCylinder3DReadOnly other, double epsilon)
   {
      return FrameCylinder3DReadOnly.super.epsilonEquals(other, epsilon) && STPCylinder3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSTPCylinder3DReadOnly other, double epsilon)
   {
      return FrameCylinder3DReadOnly.super.geometricallyEquals(other, epsilon) && STPCylinder3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

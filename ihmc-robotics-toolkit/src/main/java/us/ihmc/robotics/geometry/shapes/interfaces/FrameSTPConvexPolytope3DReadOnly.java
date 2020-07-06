package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;

public interface FrameSTPConvexPolytope3DReadOnly extends STPConvexPolytope3DReadOnly, FrameConvexPolytope3DReadOnly
{

   default boolean equals(FrameSTPConvexPolytope3DReadOnly other)
   {
      return FrameConvexPolytope3DReadOnly.super.equals(other) && STPConvexPolytope3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(FrameSTPConvexPolytope3DReadOnly other, double epsilon)
   {
      return FrameConvexPolytope3DReadOnly.super.epsilonEquals(other, epsilon) && STPConvexPolytope3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSTPConvexPolytope3DReadOnly other, double epsilon)
   {
      return FrameConvexPolytope3DReadOnly.super.geometricallyEquals(other, epsilon) && STPConvexPolytope3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}

package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;

/**
 * Read-only interface for a convex polytope that implements the sphere-torus-patches (STP) method
 * to make shapes strictly convex and that is expressed in a reference frame.
 * <p>
 * <strong> WARNING: STP convex polytope does not properly cover all scenarios and may result in a
 * non-convex shape. A STP convex polytope should always be visualized first and validate its
 * geometry, see the examples in the <i>simulation-construction-set-visualizers</i> repository. For
 * now, it is recommended to stick with primitive shapes. </strong>
 * </p>
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
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

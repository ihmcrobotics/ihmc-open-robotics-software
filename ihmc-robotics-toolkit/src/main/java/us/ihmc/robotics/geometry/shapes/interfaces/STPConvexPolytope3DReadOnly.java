package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a convex polytope that implements the sphere-torus-patches (STP) method
 * to make shapes strictly convex.
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
public interface STPConvexPolytope3DReadOnly extends STPShape3DReadOnly, ConvexPolytope3DReadOnly
{
   // This is to ensure that the default method is being overridden.
   @Override
   boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack);

   default boolean equals(STPConvexPolytope3DReadOnly other)
   {
      if (!ConvexPolytope3DReadOnly.super.equals(other))
         return false;
      return getMinimumMargin() == other.getMinimumMargin() && getMaximumMargin() == other.getMaximumMargin();
   }

   default boolean epsilonEquals(STPConvexPolytope3DReadOnly other, double epsilon)
   {
      if (!ConvexPolytope3DReadOnly.super.epsilonEquals(other, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMinimumMargin(), other.getMinimumMargin(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMaximumMargin(), other.getMaximumMargin(), epsilon))
         return false;
      return true;
   }

   default boolean geometricallyEquals(STPConvexPolytope3DReadOnly other, double epsilon)
   {
      if (!ConvexPolytope3DReadOnly.super.geometricallyEquals(other, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMinimumMargin(), other.getMinimumMargin(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMaximumMargin(), other.getMaximumMargin(), epsilon))
         return false;
      return true;
   }

   // The following part of the API has not been implemented for STP convex polytopes yet, let's prevent their use for now.

   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return STPShape3DReadOnly.super.evaluatePoint3DCollision(pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return STPShape3DReadOnly.super.signedDistance(point);
   }

   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return STPShape3DReadOnly.super.isPointInside(query, epsilon);
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return STPShape3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }
}

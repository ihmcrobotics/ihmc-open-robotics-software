package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a ramp that implements the sphere-torus-patches (STP) method to make
 * shapes strictly convex.
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public interface STPRamp3DReadOnly extends STPShape3DReadOnly, Ramp3DReadOnly
{
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      // TODO Naive implementation of the bounding box. It is guaranteed to contain the shape but it is not the tightest bounding box.
      Ramp3DReadOnly.super.getBoundingBox(boundingBoxToPack);
      boundingBoxToPack.getMinPoint().sub(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
      boundingBoxToPack.getMaxPoint().add(getMaximumMargin(), getMaximumMargin(), getMaximumMargin());
   }

   // This is to ensure that the default method is being overridden.
   @Override
   boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack);

   default boolean equals(STPRamp3DReadOnly other)
   {
      if (!Ramp3DReadOnly.super.equals(other))
         return false;
      return getMinimumMargin() == other.getMinimumMargin() && getMaximumMargin() == other.getMaximumMargin();
   }

   default boolean epsilonEquals(STPRamp3DReadOnly other, double epsilon)
   {
      if (!Ramp3DReadOnly.super.epsilonEquals(other, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMinimumMargin(), other.getMinimumMargin(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMaximumMargin(), other.getMaximumMargin(), epsilon))
         return false;
      return true;
   }

   default boolean geometricallyEquals(STPRamp3DReadOnly other, double epsilon)
   {
      if (!Ramp3DReadOnly.super.geometricallyEquals(other, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMinimumMargin(), other.getMinimumMargin(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMaximumMargin(), other.getMaximumMargin(), epsilon))
         return false;
      return true;
   }

   // The following part of the API has not been implemented for STP ramp yet, let's prevent their use for now.

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
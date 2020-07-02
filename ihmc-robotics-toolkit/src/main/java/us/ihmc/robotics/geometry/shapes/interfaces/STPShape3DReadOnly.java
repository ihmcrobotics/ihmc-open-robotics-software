package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface STPShape3DReadOnly extends Shape3DReadOnly
{
   double getMinimumMargin();

   double getMaximumMargin();

   double getSmallRadius();

   double getLargeRadius();

   // This is to ensure that the default method is being overridden.
   @Override
   boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack);

   // The following part of the API has not been implemented for STP shapes yet, let's prevent their use for now.

   @Override
   default double getVolume()
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }
}

package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.geometry.transformables.AbstractPose;
import us.ihmc.robotics.math.Epsilons;

public abstract class Shape3d<S extends Shape3d<S>> extends AbstractPose implements GeometryObject<S>
{
   /**
    * Find the distance from the closest point on this shape to the given point. Returns 0.0 if the point is inside.
    *
    * @param point
    * @return distance from the point to this Shape3d.
    */
   public final double distance(Point3DBasics point)
   {
      transformToLocal(point);
      double distance = distanceShapeFrame(point);
      transformToWorld(point);
      return distance;
   }

   protected abstract double distanceShapeFrame(Point3DReadOnly point);

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    *
    * @param pointToCheck
    * @return true if the point is inside or on the surface, false otherwise.
    */
   public final boolean isInsideOrOnSurface(Point3DBasics pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, Epsilons.ONE_TRILLIONTH);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level. If epsilonToGrowObject is positive, then 
    * the object will be checked as being larger. If negative, then the object will be shrunk.
    *
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public final boolean isInsideOrOnSurface(Point3DBasics pointToCheck, double epsilonToGrowObject)
   {
      transformToLocal(pointToCheck);
      boolean isInsideOrOnSurface = isInsideOrOnSurfaceShapeFrame(pointToCheck, epsilonToGrowObject);
      transformToWorld(pointToCheck);
      return isInsideOrOnSurface;
   }

   protected abstract boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly pointToCheck, double epsilonToGrowObject);

   /**
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * If you wish to know the surface normal, then subtract the projected point from the original point.
    *
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting orthogonal point).
    */
   public final void orthogonalProjection(Point3DBasics pointToCheckAndPack)
   {
      transformToLocal(pointToCheckAndPack);
      orthogonalProjectionShapeFrame(pointToCheckAndPack);
      transformToWorld(pointToCheckAndPack);
   }

   protected abstract void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack);

   /**
    * Returns true if inside the Shape3d. If inside, must pack the intersection and normal. If not inside, packing those is optional.
    * But if they are not packed when outside, then they should be set to NaN. If they are set to NaN and you really do wish to see
    * where they would project to, then call orthogonalProjection.
    *
    * @param pointToCheck
    * @param intersectionToPack
    * @param normalToPack
    * @return true if the point is inside, false otherwise.
    */
   public final boolean checkIfInside(Point3DBasics pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      transformToLocal(pointToCheck);
      boolean isInside = checkIfInsideShapeFrame(pointToCheck, closestPointOnSurfaceToPack, normalToPack);
      //TODO: This modifies pointToCheck and transforms back. Should we make a temp variable instead, or are we trying to be Thread safe here?
      transformToWorld(pointToCheck);
      if (closestPointOnSurfaceToPack != null)
      {
         transformToWorld(closestPointOnSurfaceToPack);
      }
      if (normalToPack != null)
      {
         transformToWorld(normalToPack);
      }
      return isInside;
   }

   protected abstract boolean checkIfInsideShapeFrame(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack);
}
